// Minimal G1 connection test
// Just subscribes to robot state and prints joint angles.
// If this works, the SDK connection is good.
//
// Build: make test_connection
// Run:   ./bin/test_connection <network_interface>
//        ./bin/test_connection eth0

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <network_interface>" << std::endl;
        std::cout << "Example: " << argv[0] << " eth0" << std::endl;
        return 1;
    }

    std::string net = argv[1];

    // Init DDS
    std::cout << "Connecting via " << net << "..." << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(0, net);

    // Subscribe to state
    unitree_hg::msg::dds_::LowState_ state;
    bool received = false;

    auto sub = std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>("rt/lowstate");
    sub->InitChannel([&](const void* msg) {
        auto s = (const unitree_hg::msg::dds_::LowState_*)msg;
        memcpy(&state, s, sizeof(unitree_hg::msg::dds_::LowState_));
        received = true;
    }, 1);

    // Wait for data
    std::cout << "Waiting for robot state..." << std::endl;
    auto start = std::chrono::steady_clock::now();
    while (!received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        float elapsed = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > 5.0f) {
            std::cerr << "FAIL: No data after 5s. Check:" << std::endl;
            std::cerr << "  - Network cable connected?" << std::endl;
            std::cerr << "  - IP in same subnet? (robot: 192.168.123.161)" << std::endl;
            std::cerr << "  - Robot powered on?" << std::endl;
            std::cerr << "  - Correct interface? (try: ip addr show)" << std::endl;
            return 1;
        }
    }

    std::cout << "\n=== CONNECTED ===" << std::endl;

    // Print state a few times
    for (int sample = 0; sample < 5; sample++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        std::cout << "\n--- Sample " << sample + 1 << " ---" << std::endl;

        // Leg joints (0-11)
        std::cout << "Legs:   ";
        for (int i = 0; i < 12; i++) {
            printf("%+.3f ", state.motor_state().at(i).q());
        }
        std::cout << std::endl;

        // Waist (12-14)
        std::cout << "Waist:  ";
        for (int i = 12; i < 15; i++) {
            printf("%+.3f ", state.motor_state().at(i).q());
        }
        std::cout << std::endl;

        // Arms (15-28)
        std::cout << "L Arm:  ";
        for (int i = 15; i < 22; i++) {
            printf("%+.3f ", state.motor_state().at(i).q());
        }
        std::cout << std::endl;

        std::cout << "R Arm:  ";
        for (int i = 22; i < 29; i++) {
            printf("%+.3f ", state.motor_state().at(i).q());
        }
        std::cout << std::endl;
    }

    std::cout << "\nConnection test PASSED." << std::endl;
    return 0;
}
