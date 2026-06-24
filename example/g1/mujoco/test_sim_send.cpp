// Simple DDS command sender for testing mujoco_sim_bridge
// Sends joint targets directly via DDS, no LocoClient needed
//
// Usage: ./test_sim_send [network_interface]

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

int main(int argc, char** argv) {
    std::string net = "lo";
    if (argc > 1) net = argv[1];

    unitree::robot::ChannelFactory::Instance()->Init(0, net);

    // Publisher: send commands to bridge
    auto pub = std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>("rt/user_lowcmd");
    pub->InitChannel();

    // Subscriber: read simulated state
    unitree_hg::msg::dds_::LowState_ state_msg;
    auto sub = std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>("rt/lowstate");
    sub->InitChannel([&](const void* msg) {
        auto s = (const unitree_hg::msg::dds_::LowState_*)msg;
        memcpy(&state_msg, s, sizeof(unitree_hg::msg::dds_::LowState_));
    }, 1);

    unitree_hg::msg::dds_::LowCmd_ cmd;

    // Default standing pose
    float standing[29] = {
        -0.2f, 0, 0, 0.4f, -0.2f, 0,   // left leg
        -0.2f, 0, 0, 0.4f, -0.2f, 0,   // right leg
        0, 0, 0,                          // waist
        0, 0, 0, 0.4f, 0, 1.2f, 0,     // left arm
        0, -0.4f, 0, 1.2f, 0, 0, 0     // right arm
    };

    // Target: raise left arm
    float target[29];
    for (int i = 0; i < 29; i++) target[i] = standing[i];
    target[15] = -1.0f;  // left_shoulder_pitch: -57°
    target[18] = 1.0f;   // left_elbow: 57°

    float kp = 60.0f;
    float kd = 1.5f;

    std::cout << "Sending commands to bridge..." << std::endl;
    std::cout << "Target: left_shoulder_pitch=" << target[15]
              << " left_elbow=" << target[18] << std::endl;

    auto start = std::chrono::steady_clock::now();

    for (int step = 0; step < 3000; step++) {
        float phase = std::min(1.0f, step / 500.0f);  // 0.5s transition

        for (int j = 0; j < 29; j++) {
            float des = standing[j] * (1 - phase) + target[j] * phase;
            cmd.motor_cmd().at(j).q(des);
            cmd.motor_cmd().at(j).dq(0);
            cmd.motor_cmd().at(j).kp(kp);
            cmd.motor_cmd().at(j).kd(kd);
            cmd.motor_cmd().at(j).tau(0);
        }
        pub->Write(cmd);

        if (step % 500 == 0) {
            float elapsed = std::chrono::duration<float>(
                std::chrono::steady_clock::now() - start).count();
            std::cout << "  t=" << elapsed << "s  LShoulderPitch_cmd=" << (standing[15] * (1-phase) + target[15] * phase)
                      << "  actual=" << state_msg.motor_state().at(15).q()
                      << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "Done. Commands sent." << std::endl;
    return 0;
}
