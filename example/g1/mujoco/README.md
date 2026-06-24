# CSV Motion Replay for G1

从 CSV 关节数据到 MuJoCo 仿真 / SDK 真机执行的完整工具链。

## 目录结构

```
mujoco/
├── csv_replay_mujoco.py        # MuJoCo 仿真回放 (Python)
├── g1_csv_replay_example.cpp   # SDK 真机执行 (C++)
├── requirements.txt            # Python 依赖
├── README.md
└── assets/g1/                  # G1 MuJoCo 模型（自包含）
    ├── g1_sim2sim_29dof.xml    # 29 DOF 仿真模型
    └── meshes/                 # STL 网格文件
```

## 快速开始

### 1. 安装 Python 依赖

```bash
pip install -r requirements.txt
```

### 2. MuJoCo 仿真回放

```bash
# 基本回放（直接位置控制）
python csv_replay_mujoco.py --csv path/to/motion.csv

# PD 力矩控制模式（更接近真机动力学）
python csv_replay_mujoco.py --csv path/to/motion.csv --mode pd

# 上肢动作，下肢保持站立
python csv_replay_mujoco.py --csv path/to/motion.csv --hold_lower

# 循环播放，0.5 倍速
python csv_replay_mujoco.py --csv path/to/motion.csv --loop --speed 0.5

# 录制回放轨迹
python csv_replay_mujoco.py --csv path/to/motion.csv --record output.npy
```

### 3. SDK 真机执行

```bash
# 编译
cd unitree_sdk2/build
cmake .. && make g1_csv_replay_example

# 执行（需要连接真机）
./g1_csv_replay_example eth0 path/to/motion.csv 60
./g1_csv_replay_example eth0 path/to/motion.csv 60 --hold-lower
```

## CSV 数据格式

LAFAN1 retargeting 格式，每帧 36 列：

| 列范围 | 内容 | 说明 |
|--------|------|------|
| 0-2 | Root XYZ | 根节点位置（上肢动作通常恒定） |
| 3-6 | Root QX,QY,QZ,QW | 根节点四元数（通常恒定） |
| 7-35 | 29 关节角度 | 弧度，与 SDK JointIndex 顺序一致 |

### 关节映射（列 7-35 → JointIndex 0-28）

| Index | 关节名 | 列 |
|-------|--------|-----|
| 0-5 | 左腿 (HipPitch/Roll/Yaw, Knee, AnklePitch/Roll) | 7-12 |
| 6-11 | 右腿 (同上) | 13-18 |
| 12-14 | 腰部 (Yaw/Roll/Pitch) | 19-21 |
| 15-21 | 左臂 (ShoulderPitch/Roll/Yaw, Elbow, WristRoll/Pitch/Yaw) | 22-28 |
| 22-28 | 右臂 (同上) | 29-35 |

## 控制模式

| 模式 | 说明 | 适用场景 |
|------|------|----------|
| `position` | 直接设置关节位置（默认） | 快速预览动作，验证运动学 |
| `pd` | PD 力矩控制 | 动力学验证（小幅度动作） |

> **注意**: LAFAN1 retargeting 数据仅考虑运动学约束，未包含动力学约束。
> 在 PD 模式下，大范围上肢运动（如作揖、大幅挥手）可能导致机器人失稳倾倒。
> 这是预期行为——真机有平衡控制器，简单的 PD 无法模拟。
> 建议先用 `position` 模式预览动作，再用 `pd` 模式验证小幅度动作的动力学可行性。

## MuJoCo 操作

- **空格键**: 暂停/继续
- **ESC**: 退出
- **鼠标左键拖拽**: 旋转视角
- **鼠标滚轮**: 缩放
- **鼠标右键拖拽**: 平移

## SDK 执行流程

1. **检查 FSM**: 确认机器人处于 PASSIVE 模式 (fsm_id=1)
2. **UserCtrl**: 切换到用户控制模式
3. **过渡**: 从当前姿态平滑过渡到 CSV 首帧（2 秒）
4. **回放**: 逐帧发送关节目标（60Hz）
5. **交还**: 切换回 InternalCtrl

## PD 增益

| 场景 | Kp | Kd | 说明 |
|------|----|----|------|
| MuJoCo PD 模式 | 80-300 | 1-10 | 高增益，快速响应 |
| SDK 真机 | 60 | 1.5 | 低增益，安全柔和 |
