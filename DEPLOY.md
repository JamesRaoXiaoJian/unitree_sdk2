# G1 上肢关键帧回放 — 部署指南

## 开发流程

Fork 仓库 → 二次开发 → 保持独立，不提交回源仓库。这是标准做法。

```
原始仓库: github.com/unitreerobotics/unitree_sdk2
    ↓ Fork
你的仓库: github.com/JamesRaoXiaoJian/unitree_sdk2
    ↓ 二次开发，自由修改
    ↓ 不需要提 PR 回源
```

## 环境配置

### 系统要求

- Ubuntu 20.04 / 22.04 (x86_64 或 aarch64)
- GCC 9.4+
- CMake 3.5+

### 一键安装依赖

```bash
sudo apt-get update && sudo apt-get install -y \
  cmake g++ build-essential \
  libyaml-cpp-dev libeigen3-dev \
  libboost-all-dev libspdlog-dev libfmt-dev
```

### 克隆仓库

```bash
git clone https://github.com/JamesRaoXiaoJian/unitree_sdk2.git
cd unitree_sdk2
```

### 编译

```bash
mkdir -p build && cd build
cmake ..
make g1_csv_replay_example test_connection
```

编译产物在 `build/bin/` 目录下。

## 网络配置

### 连接机器人

1. 用网线连接 G1 机器人
2. 设置本机 IP 与机器人同网段：

```bash
# 查看网卡名
ip addr show

# 设置 IP（机器人默认 192.168.123.161）
sudo ip addr add 192.168.123.100/24 dev eth0

# 测试连通
ping 192.168.123.161
```

### 网卡名对照

| 网卡名 | 说明 |
|--------|------|
| `eth0` | 有线网卡（常见） |
| `enp3s0` | 有线网卡（Ubuntu 命名规则） |
| `enx*` | USB 网卡 |

## 执行

### 1. 连接测试

```bash
cd build
./bin/test_connection eth0
```

预期输出：

```
Connecting via eth0...
Waiting for robot state...

=== CONNECTED ===

--- Sample 1 ---
Legs:   -0.200 +0.000 +0.000 +0.400 -0.200 +0.000 ...
Waist:  +0.000 +0.000 +0.000
L Arm:  +0.000 +0.000 +0.000 +0.400 +0.000 +1.200 +0.000
R Arm:  +0.000 -0.400 +0.000 +1.200 +0.000 +0.000 +0.000

Connection test PASSED.
```

### 2. 执行动作

```bash
# 作揖（60Hz，匹配 CSV 帧率）
./bin/g1_csv_replay_example eth0 ../作揖.csv

# 打招呼
./bin/g1_csv_replay_example eth0 ../关键帧-打招呼.csv

# 指定帧率
./bin/g1_csv_replay_example eth0 ../作揖.csv 50
```

### 执行流程

```
① 连接机器人（DDS）
② 等待状态数据（5秒超时）
③ 读取当前关节角度
④ weight 0→1（1秒，接管上肢控制）
⑤ 平滑过渡到 CSV 首帧（2秒）
⑥ 逐帧回放关键帧
⑦ weight 1→0（2秒，交还内置控制）
```

## CSV 数据格式

每行 36 列，无表头：

```
列 0-2:   根节点位置 XYZ（忽略）
列 3-6:   根节点四元数（忽略）
列 7-35:  29 个关节角度（弧度）
```

上肢关节映射（列 22-36 = SDK 关节 12-28）：

| 列 | SDK Index | 关节 |
|----|-----------|------|
| 19 | 12 | WaistYaw |
| 20 | 13 | WaistRoll |
| 21 | 14 | WaistPitch |
| 22 | 15 | LeftShoulderPitch |
| 23 | 16 | LeftShoulderRoll |
| 24 | 17 | LeftShoulderYaw |
| 25 | 18 | LeftElbow |
| 26 | 19 | LeftWristRoll |
| 27 | 20 | LeftWristPitch |
| 28 | 21 | LeftWristYaw |
| 29 | 22 | RightShoulderPitch |
| 30 | 23 | RightShoulderRoll |
| 31 | 24 | RightShoulderYaw |
| 32 | 25 | RightElbow |
| 33 | 26 | RightWristRoll |
| 34 | 27 | RightWristPitch |
| 35 | 28 | RightWristYaw |

## 安全事项

- 首次运行建议有人在旁边
- 遥控器 L2+B 急停
- 动作幅度大先在仿真里验证
- 确保周围有足够空间（手臂展开约 1.5m）

## 目录结构

```
unitree_sdk2/
├── build/
│   └── bin/
│       ├── g1_csv_replay_example   # 主程序
│       └── test_connection         # 连接测试
├── example/g1/mujoco/
│   ├── csv_replay_mujoco.py        # MuJoCo 仿真（可选）
│   ├── assets/g1/                  # MuJoCo 模型
│   └── README.md                   # 仿真文档
├── 作揖.csv                         # 作揖动作数据
├── 关键帧-打招呼.csv                 # 打招呼动作数据
└── DEPLOY.md                       # 本文档
```

## 常见问题

### 连接超时

```
ERROR: No state data received after 5s.
```

检查：
- 网线是否插好
- IP 是否在同一网段
- 机器人是否开机
- 网卡名是否正确（`ip addr show`）

### 动作不动

- 确认机器人不是在行走模式（FSM id=500）
- arm_sdk 方式不需要 PASSIVE 模式，但机器人需要在站立状态

### macOS

SDK 仅支持 Linux。macOS 需用 Docker：

```bash
docker run -it --net=host -v $(pwd):/workspace ubuntu:22.04 bash
# 容器内安装依赖、编译、执行
```
