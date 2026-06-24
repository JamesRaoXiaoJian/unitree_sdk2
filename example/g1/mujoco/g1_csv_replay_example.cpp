// CSV Joint Motion Replay for Unitree G1 — Arm SDK Mode
// ======================================================
//
// Sends upper-body keyframes to the robot via `rt/arm_sdk` topic.
// Uses weight mechanism: no PASSIVE mode required, works from any state.
//
// Formula: Motor_real = weight * User_Cmd + (1 - weight) * BuiltIn_Cmd
//
// Flow:
//   1. Connect DDS
//   2. Read current joint positions
//   3. weight 0→1 (engage user control)
//   4. Smooth transition to CSV first frame
//   5. Replay CSV keyframes at 50 Hz
//   6. weight 1→0 (release to built-in control)
//
// Usage:
//   ./g1_csv_replay_example <net> <csv> [fps]
//   ./g1_csv_replay_example eth0 motion.csv 50

#include <algorithm>
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

static const std::string kTopicArmSDK = "rt/arm_sdk";
static const std::string kTopicState = "rt/lowstate";

// Upper-body joint indices (arms + waist = 17 DOF)
enum ArmJointIndex {
    kLeftShoulderPitch = 15,
    kLeftShoulderRoll = 16,
    kLeftShoulderYaw = 17,
    kLeftElbow = 18,
    kLeftWristRoll = 19,
    kLeftWristPitch = 20,
    kLeftWristYaw = 21,
    kRightShoulderPitch = 22,
    kRightShoulderRoll = 23,
    kRightShoulderYaw = 24,
    kRightElbow = 25,
    kRightWristRoll = 26,
    kRightWristPitch = 27,
    kRightWristYaw = 28,
    kWaistYaw = 12,
    kWaistRoll = 13,
    kWaistPitch = 14,
    kNotUsedJoint = 29,  // Used to send weight
};

// Joint order for arm_sdk (17 joints)
static const std::array<int, 17> kArmJoints = {
    15, 16, 17, 18, 19, 20, 21,  // left arm
    22, 23, 24, 25, 26, 27, 28,  // right arm
    12, 13, 14,                   // waist
};

static constexpr float kDefaultKp = 60.0f;
static constexpr float kDefaultKd = 1.5f;
static constexpr float kMaxJointVelocity = 0.5f;  // rad/s

struct CsvFrame {
    std::array<float, 29> joints;  // Full 29 DOF from CSV
};

std::vector<CsvFrame> LoadCsv(const std::string& path) {
    std::vector<CsvFrame> frames;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot open " << path << std::endl;
        return frames;
    }

    std::string line;
    while (std::getline(file, line)) {
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
        for (int i = 0; i < 29; i++) frame.joints[i] = values[7 + i];
        frames.push_back(frame);
    }

    std::cout << "Loaded " << frames.size() << " frames ("
              << frames.size() / 60.0f << "s)" << std::endl;
    return frames;
}

int main(int argc, char const* argv[]) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <net> <csv> [fps]" << std::endl;
        return 1;
    }

    std::string net = argv[1];
    std::string csv_path = argv[2];
    float fps = 60.0f;  // Default 60 Hz to match CSV data rate
    for (int i = 3; i < argc; i++) {
        try { fps = std::stof(argv[i]); } catch (...) {}
    }

    auto frames = LoadCsv(csv_path);
    if (frames.empty()) return 1;

    // ---- DDS Init ----
    std::cout << "Connecting via " << net << "..." << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(0, net);

    auto arm_pub = std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(kTopicArmSDK);
    arm_pub->InitChannel();

    unitree_hg::msg::dds_::LowState_ state_msg;
    std::atomic<bool> state_received{false};
    auto state_sub = std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(kTopicState);
    state_sub->InitChannel([&](const void* msg) {
        auto s = (const unitree_hg::msg::dds_::LowState_*)msg;
        memcpy(&state_msg, s, sizeof(unitree_hg::msg::dds_::LowState_));
        state_received = true;
    }, 1);

    // Wait for state
    std::cout << "Waiting for robot state..." << std::endl;
    auto t0 = std::chrono::steady_clock::now();
    while (!state_received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count() > 5.0f) {
            std::cerr << "ERROR: No state after 5s." << std::endl;
            return 1;
        }
    }
    std::cout << "Connected." << std::endl;

    // ---- Read current positions ----
    std::array<float, 17> current_jpos{};
    for (int i = 0; i < 17; i++) {
        current_jpos[i] = state_msg.motor_state().at(kArmJoints[i]).q();
    }

    // ---- Parameters ----
    unitree_hg::msg::dds_::LowCmd_ cmd_msg;
    float control_dt = 1.0f / fps;
    float max_joint_delta = kMaxJointVelocity / fps;
    auto sleep_time = std::chrono::microseconds(static_cast<int>(control_dt * 1000000));

    // Weight engage/disengage
    float weight = 0.0f;
    float weight_rate = 0.2f;           // weight change per second
    float delta_weight = weight_rate * control_dt;

    // ---- Phase 1: Engage weight 0→1 ----
    std::cout << "=== Phase 1: Engage (weight 0→1) ===" << std::endl;
    float engage_time = 1.0f;
    int engage_steps = static_cast<int>(engage_time / control_dt);

    for (int i = 0; i < engage_steps; i++) {
        weight = std::clamp(weight + delta_weight, 0.0f, 1.0f);
        cmd_msg.motor_cmd().at(kNotUsedJoint).q(weight);

        // Send current positions during engage (no movement)
        for (int j = 0; j < 17; j++) {
            cmd_msg.motor_cmd().at(kArmJoints[j]).q(current_jpos[j]);
            cmd_msg.motor_cmd().at(kArmJoints[j]).dq(0);
            cmd_msg.motor_cmd().at(kArmJoints[j]).kp(kDefaultKp);
            cmd_msg.motor_cmd().at(kArmJoints[j]).kd(kDefaultKd);
            cmd_msg.motor_cmd().at(kArmJoints[j]).tau(0);
        }
        arm_pub->Write(cmd_msg);
        std::this_thread::sleep_for(sleep_time);
    }
    std::cout << "  weight = " << weight << std::endl;

    // ---- Phase 2: Transition to CSV first frame ----
    std::cout << "=== Phase 2: Transition to initial pose ===" << std::endl;

    // Extract upper-body targets from CSV frame 0
    std::array<float, 17> first_target{};
    for (int i = 0; i < 17; i++) {
        first_target[i] = frames[0].joints[kArmJoints[i] - 12 + 12];
        // kArmJoints[i] is the SDK joint index (12-28)
        // CSV joints[kArmJoints[i]] is the corresponding CSV value
    }
    // Simpler: just use the SDK index directly
    for (int i = 0; i < 17; i++) {
        first_target[i] = frames[0].joints[kArmJoints[i]];
    }

    std::array<float, 17> cmd_pos = current_jpos;
    float transition_time = 2.0f;
    int transition_steps = static_cast<int>(transition_time / control_dt);

    for (int i = 0; i < transition_steps; i++) {
        weight = std::clamp(weight + delta_weight, 0.0f, 1.0f);
        cmd_msg.motor_cmd().at(kNotUsedJoint).q(weight);

        for (int j = 0; j < 17; j++) {
            float delta = std::clamp(first_target[j] - cmd_pos[j], -max_joint_delta, max_joint_delta);
            cmd_pos[j] += delta;
            cmd_msg.motor_cmd().at(kArmJoints[j]).q(cmd_pos[j]);
            cmd_msg.motor_cmd().at(kArmJoints[j]).dq(0);
            cmd_msg.motor_cmd().at(kArmJoints[j]).kp(kDefaultKp);
            cmd_msg.motor_cmd().at(kArmJoints[j]).kd(kDefaultKd);
            cmd_msg.motor_cmd().at(kArmJoints[j]).tau(0);
        }
        arm_pub->Write(cmd_msg);
        std::this_thread::sleep_for(sleep_time);
    }
    std::cout << "Transition complete." << std::endl;

    // ---- Phase 3: Replay CSV ----
    std::cout << "=== Phase 3: Replaying " << frames.size() << " frames ===" << std::endl;

    auto replay_start = std::chrono::steady_clock::now();
    for (size_t fi = 0; fi < frames.size(); fi++) {
        auto frame_start = std::chrono::steady_clock::now();

        cmd_msg.motor_cmd().at(kNotUsedJoint).q(weight);

        for (int j = 0; j < 17; j++) {
            float target = frames[fi].joints[kArmJoints[j]];
            float delta = std::clamp(target - cmd_pos[j], -max_joint_delta, max_joint_delta);
            cmd_pos[j] += delta;

            cmd_msg.motor_cmd().at(kArmJoints[j]).q(cmd_pos[j]);
            cmd_msg.motor_cmd().at(kArmJoints[j]).dq(0);
            cmd_msg.motor_cmd().at(kArmJoints[j]).kp(kDefaultKp);
            cmd_msg.motor_cmd().at(kArmJoints[j]).kd(kDefaultKd);
            cmd_msg.motor_cmd().at(kArmJoints[j]).tau(0);
        }
        arm_pub->Write(cmd_msg);

        if (fi % 60 == 0) {
            float t = std::chrono::duration<float>(
                std::chrono::steady_clock::now() - replay_start).count();
            std::cout << "  frame " << fi << "/" << frames.size()
                      << " t=" << t << "s" << std::endl;
        }

        auto elapsed = std::chrono::steady_clock::now() - frame_start;
        if (elapsed < sleep_time) std::this_thread::sleep_for(sleep_time - elapsed);
    }

    float total = std::chrono::duration<float>(
        std::chrono::steady_clock::now() - replay_start).count();
    std::cout << "Replay done: " << frames.size() << " frames in " << total << "s" << std::endl;

    // ---- Phase 4: Disengage weight 1→0 ----
    std::cout << "=== Phase 4: Disengage (weight 1→0) ===" << std::endl;
    float disengage_time = 2.0f;
    int disengage_steps = static_cast<int>(disengage_time / control_dt);

    for (int i = 0; i < disengage_steps; i++) {
        weight = std::clamp(weight - delta_weight, 0.0f, 1.0f);
        cmd_msg.motor_cmd().at(kNotUsedJoint).q(weight);
        arm_pub->Write(cmd_msg);
        std::this_thread::sleep_for(sleep_time);
    }

    // Final zero weight
    cmd_msg.motor_cmd().at(kNotUsedJoint).q(0);
    arm_pub->Write(cmd_msg);

    std::cout << "Done. Robot returned to built-in control." << std::endl;
    return 0;
}
