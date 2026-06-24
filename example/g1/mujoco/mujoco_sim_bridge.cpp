// MuJoCo DDS Simulation Bridge for Unitree G1
// ==============================================
//
// Headless bridge: subscribes to SDK DDS commands, runs MuJoCo physics,
// publishes simulated state back. No GUI — use csv_replay_mujoco.py for visualization.
//
// Data flow:
//   g1_csv_replay_example → DDS (rt/user_lowcmd) → THIS → MuJoCo → DDS (rt/lowstate)
//
// Usage:
//   ./mujoco_sim_bridge --xml assets/g1/g1_sim2sim_29dof.xml [--net lo] [--hz 1000]

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <mujoco/mujoco.h>

static const std::string kTopicCmd = "rt/user_lowcmd";
static const std::string kTopicState = "rt/lowstate";
static constexpr int kNumJoints = 29;
static constexpr int kNumMotors = 35;

// Shared between DDS callback and physics thread
struct SimState {
    std::mutex mtx;
    double q_target[kNumMotors] = {};
    double dq_target[kNumMotors] = {};
    double kp[kNumMotors] = {};
    double kd[kNumMotors] = {};
    double tau_ff[kNumMotors] = {};
    std::atomic<bool> new_cmd{false};
    std::atomic<bool> running{true};
};

SimState g_state;

// DDS callback
void on_lowcmd(const void* msg) {
    auto cmd = (const unitree_hg::msg::dds_::LowCmd_*)msg;
    std::lock_guard<std::mutex> lock(g_state.mtx);
    for (int i = 0; i < kNumMotors; i++) {
        g_state.q_target[i] = cmd->motor_cmd().at(i).q();
        g_state.dq_target[i] = cmd->motor_cmd().at(i).dq();
        g_state.kp[i] = cmd->motor_cmd().at(i).kp();
        g_state.kd[i] = cmd->motor_cmd().at(i).kd();
        g_state.tau_ff[i] = cmd->motor_cmd().at(i).tau();
    }
    g_state.new_cmd = true;
}

int main(int argc, char** argv) {
    std::string xml_path;
    int hz = 1000;
    std::string net = "lo";

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--xml" && i + 1 < argc) xml_path = argv[++i];
        else if (arg == "--hz" && i + 1 < argc) hz = std::stoi(argv[++i]);
        else if (arg == "--net" && i + 1 < argc) net = argv[++i];
        else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0]
                      << " --xml <model.xml> [--hz 1000] [--net lo]" << std::endl;
            return 0;
        }
    }

    if (xml_path.empty()) {
        std::cerr << "ERROR: --xml required. Example: --xml assets/g1/g1_sim2sim_29dof.xml"
                  << std::endl;
        return 1;
    }

    // ---- Load MuJoCo ----
    std::cout << "Loading: " << xml_path << std::endl;
    char err[1000] = "";
    mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
    if (!m) {
        std::cerr << "MuJoCo error: " << err << std::endl;
        return 1;
    }
    mjData* d = mj_makeData(m);

    // Reset to keyframe
    if (m->nkey > 0) {
        mj_resetDataKeyframe(m, d, 0);
        std::cout << "Keyframe 'home' loaded" << std::endl;
    } else {
        mj_resetData(m, d);
    }

    // Adjust pelvis height
    mj_forward(m, d);
    double min_z = 999;
    for (int i = 0; i < m->ngeom; i++) {
        if (m->geom_contype[i] > 0 && m->geom_bodyid[i] > 0) {
            if (d->geom_xpos[i * 3 + 2] < min_z)
                min_z = d->geom_xpos[i * 3 + 2];
        }
    }
    if (min_z > 0.001) {
        d->qpos[2] -= min_z;
        std::cout << "Adjusted pelvis Z by " << -min_z << "m" << std::endl;
    }
    d->qvel[0] = d->qvel[1] = d->qvel[2] = 0;
    mj_forward(m, d);

    std::cout << "Model: nq=" << m->nq << " nv=" << m->nv
              << " nu=" << m->nu << " nbody=" << m->nbody << std::endl;

    // ---- Init DDS ----
    unitree::robot::ChannelFactory::Instance()->Init(0, net);

    auto cmd_sub = std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>>(kTopicCmd);
    cmd_sub->InitChannel(on_lowcmd, 1);

    auto state_pub = std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowState_>>(kTopicState);
    state_pub->InitChannel();
    unitree_hg::msg::dds_::LowState_ state_msg;

    // ---- Physics loop ----
    std::cout << "\n========================================" << std::endl;
    std::cout << "MuJoCo Sim Bridge running" << std::endl;
    std::cout << "  Physics: " << hz << " Hz" << std::endl;
    std::cout << "  Subscribe: " << kTopicCmd << std::endl;
    std::cout << "  Publish:   " << kTopicState << " @ 50 Hz" << std::endl;
    std::cout << "  Ctrl+C to stop" << std::endl;
    std::cout << "========================================\n" << std::endl;

    auto dt = std::chrono::microseconds(1000000 / hz);
    int state_counter = 0;
    int64_t step_count = 0;
    auto t_start = std::chrono::steady_clock::now();

    while (g_state.running) {
        auto t0 = std::chrono::steady_clock::now();

        // Apply PD control from latest SDK command
        {
            std::lock_guard<std::mutex> lock(g_state.mtx);
            for (int i = 0; i < kNumJoints && i < m->nu; i++) {
                double pos = d->qpos[7 + i];
                double vel = d->qvel[6 + i];
                double q_des = g_state.q_target[i];
                double dq_des = g_state.dq_target[i];
                double kp_val = g_state.kp[i];
                double kd_val = g_state.kd[i];

                // PD + feedforward
                double torque = kp_val * (q_des - pos) + kd_val * (dq_des - vel) + g_state.tau_ff[i];
                d->ctrl[i] = torque;
            }
        }

        // Step physics
        mj_step(m, d);
        step_count++;

        // Publish state at 50 Hz
        state_counter++;
        if (state_counter >= hz / 50) {
            state_counter = 0;
            for (int i = 0; i < kNumMotors; i++) {
                if (i < kNumJoints) {
                    state_msg.motor_state().at(i).q(d->qpos[7 + i]);
                    state_msg.motor_state().at(i).dq(d->qvel[6 + i]);
                } else {
                    state_msg.motor_state().at(i).q(0);
                    state_msg.motor_state().at(i).dq(0);
                }
            }
            state_pub->Write(state_msg);
        }

        // Status print every 5s
        if (step_count % (hz * 5) == 0) {
            auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start).count();
            std::cout << "  t=" << std::fixed << std::setprecision(1) << elapsed
                      << "s  pelvis_z=" << std::setprecision(3) << d->qpos[2]
                      << "m  steps=" << step_count << std::endl;
        }

        // Maintain physics rate
        auto elapsed = std::chrono::steady_clock::now() - t0;
        if (elapsed < dt) {
            std::this_thread::sleep_for(dt - elapsed);
        }
    }

    // Cleanup
    mj_deleteData(d);
    mj_deleteModel(m);
    std::cout << "Sim bridge stopped. Total steps: " << step_count << std::endl;
    return 0;
}
