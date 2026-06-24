#!/usr/bin/env python3
"""
CSV Joint Motion Replay in MuJoCo for Unitree G1
=================================================

Loads LAFAN1-retargeted CSV joint data and replays it in MuJoCo simulation.
All model assets (XML + meshes) are bundled in assets/g1/ relative to this script.

CSV format (36 columns per frame, no header):
  Col 0-2:  Root position XYZ (constant for upper-body motions)
  Col 3-6:  Root quaternion QX,QY,QZ,QW (constant for upper-body motions)
  Col 7-35: 29 joint angles (radians), matching SDK JointIndex order:
            0-5:   Left leg  (HipPitch, HipRoll, HipYaw, Knee, AnklePitch, AnkleRoll)
            6-11:  Right leg (same)
            12-14: Waist    (Yaw, Roll, Pitch)
            15-21: Left arm (ShoulderPitch, Roll, Yaw, Elbow, WristRoll, Pitch, Yaw)
            22-28: Right arm (same)

Joint mapping determined by:
  1. LAFAN1 official spec: root(XYZQXQYQZQW) + joint angles
  2. First 7 cols are near-constant (root pose), remaining 29 vary per frame
  3. Order matches SDK defines.h JointIndex and MuJoCo XML joint order
  4. Value ranges are physically plausible for each joint (verified)

Usage:
  python csv_replay_mujoco.py --csv path/to/motion.csv [options]

Options:
  --xml         MuJoCo XML model path (default: assets/g1/g1_sim2sim_29dof.xml)
  --fps         Playback FPS (default: 60, matching CSV data rate)
  --mode        Control mode: 'position' (direct qpos) or 'pd' (PD torque control)
  --loop [N]    Loop playback N times (no arg = infinite loop)
  --hold_lower  Hold lower body in default standing pose (for upper-body motions)
  --record      Save joint trajectory to .npy file
  --speed       Playback speed multiplier (default: 1.0)
  --wait        Seconds to wait after playback before exit (default: 1.0)
"""

import argparse
import csv
import os
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

# ---- Resolve paths relative to this script ----
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_XML = os.path.join(SCRIPT_DIR, "assets", "g1", "g1_sim2sim_29dof.xml")

# ---- G1 Joint Configuration ----

JOINT_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

# Default standing pose (from g1_sim2sim_29dof.xml keyframe "home")
DEFAULT_STANDING = np.array([
    -0.2, 0, 0, 0.4, -0.2, 0,        # left leg
    -0.2, 0, 0, 0.4, -0.2, 0,        # right leg
    0, 0, 0,                           # waist
    0, 0, 0, 0.4, 0, 1.2, 0,         # left arm
    0, -0.4, 0, 1.2, 0, 0, 0,        # right arm
])

# PD gains (tuned for MuJoCo simulation)
PD_STIFFNESS = np.array([
    200, 200, 200, 300, 200, 200,
    200, 200, 200, 300, 200, 200,
    300, 300, 300,
    80, 80, 80, 80, 10, 10, 10,
    80, 80, 80, 80, 10, 10, 10,
])

PD_DAMPING = np.array([
    5, 5, 5, 8, 10, 10,
    5, 5, 5, 8, 10, 10,
    8, 8, 8,
    8, 8, 8, 8, 1, 1, 1,
    8, 8, 8, 8, 1, 1, 1,
])

TORQUE_LIMITS = np.array([
    200, 200, 200, 300, 300, 300,
    200, 200, 200, 300, 300, 300,
    300, 300, 300,
    100, 100, 100, 100, 20, 20, 20,
    100, 100, 100, 100, 20, 20, 20,
])

LOWER_BODY_INDICES = list(range(15))


def load_csv(csv_path):
    """Load CSV motion data.

    Returns:
        joint_frames: np.ndarray (N, 29)
        root_pos: np.ndarray (3,)
        root_quat: np.ndarray (4,) - WXYZ for MuJoCo
    """
    rows = []
    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) == 0:
                continue
            values = [float(x) for x in row]
            if len(values) != 36:
                print(f"WARNING: Expected 36 columns, got {len(values)}. Skipping.")
                continue
            rows.append(values)

    data = np.array(rows)
    n = len(rows)
    print(f"Loaded {n} frames from {os.path.basename(csv_path)}")
    print(f"  Duration: {n/60:.2f}s at 60 FPS")

    root_pos = data[0, 0:3]
    q = data[0, 3:7]  # QX,QY,QZ,QW
    root_quat = np.array([q[3], q[0], q[1], q[2]])  # WXYZ

    joint_frames = data[:, 7:36]

    print(f"  Root position: ({root_pos[0]:.3f}, {root_pos[1]:.3f}, {root_pos[2]:.3f})")
    print(f"  Active joints:")
    for i, name in enumerate(JOINT_NAMES):
        lo, hi = joint_frames[:, i].min(), joint_frames[:, i].max()
        if hi - lo > 0.01:
            print(f"    [{i:2d}] {name:30s}: [{lo:+.3f}, {hi:+.3f}] rad "
                  f"({np.degrees(lo):+.1f} ~ {np.degrees(hi):+.1f} deg)")

    return joint_frames, root_pos, root_quat


def main():
    parser = argparse.ArgumentParser(description="Replay CSV joint motion in MuJoCo")
    parser.add_argument("--csv", type=str, required=True, help="Path to CSV motion file")
    parser.add_argument("--xml", type=str, default=None,
                        help=f"MuJoCo XML model path (default: {DEFAULT_XML})")
    parser.add_argument("--fps", type=float, default=60.0, help="Playback FPS")
    parser.add_argument("--mode", type=str, default="position",
                        choices=["position", "pd"],
                        help="Control mode: 'position' or 'pd'")
    parser.add_argument("--loop", type=int, nargs="?", const=-1, default=0,
                        help="Loop N times (no arg=infinite, omit=play once)")
    parser.add_argument("--hold_lower", action="store_true",
                        help="Hold lower body in default standing pose")
    parser.add_argument("--record", type=str, default=None,
                        help="Save trajectory to .npy file")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Playback speed multiplier")
    parser.add_argument("--wait", type=float, default=1.0,
                        help="Seconds to wait after playback before exit")
    args = parser.parse_args()

    # Resolve XML path
    xml_path = args.xml or DEFAULT_XML
    if not os.path.isabs(xml_path):
        xml_path = os.path.join(SCRIPT_DIR, xml_path)
    if not os.path.exists(xml_path):
        print(f"ERROR: XML model not found: {xml_path}")
        print(f"Expected at: {DEFAULT_XML}")
        sys.exit(1)

    print(f"Loading MuJoCo model: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    joint_frames, root_pos, root_quat = load_csv(args.csv)

    # Reset to keyframe if available
    try:
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(model, data, key_id)
            print("Using keyframe 'home' as initial pose")
        else:
            mujoco.mj_resetData(model, data)
    except Exception:
        mujoco.mj_resetData(model, data)

    data.qpos[0:3] = root_pos
    data.qpos[3:7] = root_quat
    data.qpos[7:36] = joint_frames[0]
    mujoco.mj_forward(model, data)

    # Adjust pelvis height
    min_z = 999
    for i in range(model.ngeom):
        g = model.geom(i)
        if g.contype[0] > 0 and g.bodyid[0] > 0:
            if data.geom_xpos[i][2] < min_z:
                min_z = data.geom_xpos[i][2]
    if min_z > 0.001:
        data.qpos[2] -= min_z
        print(f"Adjusted pelvis height by -{min_z:.4f}m")
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)

    # Timing
    frame_dt = 1.0 / (args.fps * args.speed)
    frame_idx = 0
    total_frames = len(joint_frames)
    loop_count = 0
    done = False

    recorded_qpos = [] if args.record else None

    # Loop description
    if args.loop == -1:
        loop_desc = "infinite"
    elif args.loop > 0:
        loop_desc = f"{args.loop} times"
    else:
        loop_desc = "once (auto-exit)"

    print(f"\n{'='*60}")
    print(f"Playback: {total_frames} frames @ {args.fps} FPS (speed={args.speed}x)")
    print(f"Mode: {args.mode} | Hold lower: {args.hold_lower} | Loop: {loop_desc}")
    print(f"Controls: SPACE=pause/resume  ESC=quit")
    print(f"{'='*60}\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[2] = 0.8
        viewer.cam.distance = 2.5
        viewer.cam.elevation = -10

        paused = False
        last_time = time.time()
        done_time = 0.0

        while viewer.is_running():
            current_time = time.time()
            elapsed = current_time - last_time

            if not paused and not done and elapsed >= frame_dt:
                last_time = current_time

                target = joint_frames[frame_idx].copy()
                if args.hold_lower:
                    for i in LOWER_BODY_INDICES:
                        target[i] = DEFAULT_STANDING[i]

                if args.mode == "position":
                    data.qpos[7:36] = target
                    mujoco.mj_forward(model, data)
                else:
                    dof_pos = data.qpos[7:36]
                    dof_vel = data.qvel[6:35]
                    torque = (target - dof_pos) * PD_STIFFNESS - dof_vel * PD_DAMPING
                    torque = np.clip(torque, -TORQUE_LIMITS, TORQUE_LIMITS)
                    data.ctrl[:] = torque
                    for _ in range(max(1, int(frame_dt / model.opt.timestep))):
                        mujoco.mj_step(model, data)

                if recorded_qpos is not None:
                    recorded_qpos.append(data.qpos[7:36].copy())

                frame_idx += 1
                if frame_idx >= total_frames:
                    if args.loop == -1:
                        # Infinite loop
                        frame_idx = 0
                        loop_count += 1
                        print(f"  Loop {loop_count} complete, restarting...")
                    elif args.loop > 0:
                        # N loops
                        loop_count += 1
                        if loop_count >= args.loop:
                            print(f"  All {args.loop} loops complete")
                            done = True
                        else:
                            frame_idx = 0
                            print(f"  Loop {loop_count}/{args.loop} complete")
                    else:
                        # No loop, play once then wait and exit
                        print(f"  Playback complete ({total_frames} frames)")
                        done = True
                        done_time = time.time()

            # Auto-exit after wait seconds when done
            if done:
                if time.time() - done_time >= args.wait:
                    break

            viewer.sync()
            time.sleep(0.001)

    if recorded_qpos and args.record:
        recorded_qpos = np.array(recorded_qpos)
        np.save(args.record, recorded_qpos)
        print(f"Saved {len(recorded_qpos)} frames to {args.record}")


if __name__ == "__main__":
    main()
