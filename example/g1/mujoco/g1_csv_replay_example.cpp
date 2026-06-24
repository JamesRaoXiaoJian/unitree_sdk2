// CSV Joint Motion Replay for Unitree G1 (Real Robot)
// ====================================================
//
// Reads CSV joint data and replays on the real G1 robot via SDK.
//
// Flow:
//   1. Connect to robot via DDS
//   2. Wait for state data
//   3. Switch to UserCtrl
//   4. Smooth transition to CSV first frame
//   5. Replay all frames at target FPS
//   6. Switch back to InternalCtrl
//
// Usage:
//   ./g1_csv_replay_example <network_interface> <csv_path> [fps] [--hold-lower]
//   ./g1_csv_replay_example eth0 motion.csv 60 --hold-lower

#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

static const std::string kTopicUserCtrl = "rt/user_lowcmd";
static const std::string kTopicState = "rt/lowstate";
static constexpr int kNumJoints = 29;

// Default standing pose (from XML keyframe)
static const std::array<float, kNumJoints> kDefaultStanding = {
    -0.2f, 0.0f, 0.0f, 0.4f, -0.2f, 0.0f,    // left leg
    -0.2f, 0.0f, 0.0f, 0.4f, -0.2f, 0.0f,    // right leg
     0.0f, 0.0f, 0.0f,                         // waist
     0.0f, 0.0f, 0.0f, 0.4f, 0.0f, 1.2f, 0.0f,  // left arm
     0.0f, -0.4f, 0.0f, 1.2f, 0.0f, 0.0f, 0.0f  // right arm
};

static constexpr float kDefaultKp = 60.0f;
static constexpr float kDefaultKd = 1.5f;

struct CsvFrame {
    std::array<float, kNumJoints> joints;
};

std::vector<CsvFrame> LoadCsv(const std::string& path) {
    std::vector<CsvFrame> frames;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot open " << path << std::endl;
        return frames;
    }

    std::string line;
    int line_num = 0;
    while (std::getline(file, line)) {
        line_num++;
        if (line.empty()) continue;

        std::vector<float> values;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            try { values.push_back(std::stof(cell)); }
            catch (...) { break; }
        }

        if (values.size() != 36) continue;

        CsvFrame frame;
        for (int i = 0; i < kNumJoints; i++) {
            frame.joints[i] = values[7 + i];
        }
        frames.push_back(frame);
    }

    std::cout << "Loaded " << frames.size() << " frames (" << frames.size() / 60.0f << "s)" << std::endl;
    return frames;
}

int main(int argc, char const* argv[]) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <net> <csv> [fps] [--hold-lower]" << std::endl;
        return 1;
    }

    std::string net = argv[1];
    std::string csv_path = argv[2];
    float fps = 60.0f;
    bool hold_lower = false;

    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--hold-lower") hold_lower = true;
        else { try { fps = std::stof(arg); } catch (...) {} }
    }

    // Load CSV
    auto frames = LoadCsv(csv_path);
    if (frames.empty()) return 1;

    // ---- DDS Init ----
    std::cout << "Connecting to robot via " << net << "..." << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(0, net);

    // Publisher: send joint commands
    auto cmd_pub = std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(kTopicUserCtrl);
    cmd_pub->InitChannel();

    // Subscriber: read robot state
    unitree_hg::msg::dds_::LowState_ state_msg;
    std::atomic<bool> state_received{false};
    auto state_sub = std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(kTopicState);
    state_sub->InitChannel([&](const void* msg) {
        auto s = (const unitree_hg::msg::dds_::LowState_*)msg;
        memcpy(&state_msg, s, sizeof(unitree_hg::msg::dds_::LowState_));
        state_received = true;
    }, 1);

    // Wait for state data
    std::cout << "Waiting for robot state..." << std::endl;
    auto wait_start = std::chrono::steady_clock::now();
    while (!state_received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (std::chrono::duration<float>(std::chrono::steady_clock::now() - wait_start).count() > 5.0f) {
            std::cerr << "ERROR: No state data received after 5s. Check connection." << std::endl;
            return 1;
        }
    }
    std::cout << "State received. Joints: ";
    for (int i = 0; i < 6; i++) std::cout << state_msg.motor_state().at(i).q() << " ";
    std::cout << "..." << std::endl;

    // ---- FSM Control ----
    unitree::robot::g1::LocoClient client;
    client.Init();
    client.SetTimeout(5.0f);

    int fsm_id;
    client.GetFsmId(fsm_id);
    std::cout << "FSM id: " << fsm_id;
    if (fsm_id != 1) {
        std::cout << " (NOT PASSIVE - please switch robot to PASSIVE mode)" << std::endl;
        return 1;
    }
    std::cout << " (PASSIVE - OK)" << std::endl;

    // Read current joint positions
    std::array<float, kNumJoints> current_jpos{};
    for (int i = 0; i < kNumJoints; i++) {
        current_jpos[i] = state_msg.motor_state().at(i).q();
    }

    // ---- Phase 1: Switch to UserCtrl ----
    std::cout << "\n=== Phase 1: Switch to UserCtrl ===" << std::endl;

    // Send initial zero command
    unitree_hg::msg::dds_::LowCmd_ cmd_msg;
    for (int j = 0; j < kNumJoints; j++) {
        cmd_msg.motor_cmd().at(j).q(current_jpos[j]);
        cmd_msg.motor_cmd().at(j).dq(0);
        cmd_msg.motor_cmd().at(j).kp(0);
        cmd_msg.motor_cmd().at(j).kd(kDefaultKd);
        cmd_msg.motor_cmd().at(j).tau(0);
    }
    cmd_pub->Write(cmd_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    client.SwitchToUserCtrl();
    std::cout << "UserCtrl active." << std::endl;

    // ---- Phase 2: Smooth transition to first CSV frame ----
    std::cout << "=== Phase 2: Transition to initial pose ===" << std::endl;

    std::array<float, kNumJoints> first_target{};
    for (int j = 0; j < kNumJoints; j++) {
        first_target[j] = (hold_lower && j < 15) ? kDefaultStanding[j] : frames[0].joints[j];
    }

    auto control_dt = std::chrono::microseconds(static_cast<int>(1000000.0f / fps));
    float transition_time = 2.0f;
    int transition_steps = static_cast<int>(transition_time * fps);

    for (int i = 0; i < transition_steps; i++) {
        float phase = static_cast<float>(i) / transition_steps;
        for (int j = 0; j < kNumJoints; j++) {
            float des = current_jpos[j] * (1.0f - phase) + first_target[j] * phase;
            cmd_msg.motor_cmd().at(j).q(des);
            cmd_msg.motor_cmd().at(j).dq(0);
            cmd_msg.motor_cmd().at(j).kp(kDefaultKp);
            cmd_msg.motor_cmd().at(j).kd(kDefaultKd);
            cmd_msg.motor_cmd().at(j).tau(0);
        }
        cmd_pub->Write(cmd_msg);
        std::this_thread::sleep_for(control_dt);
    }
    std::cout << "Transition complete." << std::endl;

    // ---- Phase 3: Replay CSV ----
    std::cout << "=== Phase 3: Replaying " << frames.size() << " frames ===" << std::endl;

    auto replay_start = std::chrono::steady_clock::now();
    for (size_t fi = 0; fi < frames.size(); fi++) {
        auto frame_start = std::chrono::steady_clock::now();

        for (int j = 0; j < kNumJoints; j++) {
            float target = (hold_lower && j < 15) ? kDefaultStanding[j] : frames[fi].joints[j];
            cmd_msg.motor_cmd().at(j).q(target);
            cmd_msg.motor_cmd().at(j).dq(0);
            cmd_msg.motor_cmd().at(j).kp(kDefaultKp);
            cmd_msg.motor_cmd().at(j).kd(kDefaultKd);
            cmd_msg.motor_cmd().at(j).tau(0);
        }
        cmd_pub->Write(cmd_msg);

        if (fi % 60 == 0) {
            float t = std::chrono::duration<float>(
                std::chrono::steady_clock::now() - replay_start).count();
            std::cout << "  frame " << fi << "/" << frames.size() << " t=" << t << "s" << std::endl;
        }

        auto elapsed = std::chrono::steady_clock::now() - frame_start;
        if (elapsed < control_dt) std::this_thread::sleep_for(control_dt - elapsed);
    }

    float total = std::chrono::duration<float>(
        std::chrono::steady_clock::now() - replay_start).count();
    std::cout << "Replay done: " << frames.size() << " frames in " << total << "s" << std::endl;

    // ---- Phase 4: Return control ----
    std::cout << "=== Phase 4: Returning to InternalCtrl ===" << std::endl;
    client.SwitchToInternalCtrl(unitree::robot::g1::InternalFsmMode::LAST);
    std::cout << "Done." << std::endl;

    return 0;
}
