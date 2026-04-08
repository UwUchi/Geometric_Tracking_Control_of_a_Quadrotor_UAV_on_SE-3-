# Geometric Tracking Control of a Quadrotor UAV on SE(3)

一个基于 ROS 2 Humble 的四旋翼 SE(3) 几何跟踪控制复现项目。工作区当前包含：

- `quad_se3_msgs`：状态、控制输入、轨迹点消息定义
- `quad_se3_py`：单节点仿真、参考轨迹、RViz 可视化与离线分析

项目目标是先搭建一个可运行的论文风格闭环仿真，再通过 RViz 和 rosbag 做轨迹、姿态与误差分析。

## Current Features

- SE(3) 几何控制闭环：`sim_node` 内按固定顺序执行参考生成、控制计算、动力学积分与消息发布
- 自定义 ROS 2 消息：
  - `QuadState`
  - `ControlInput`
  - `TrajectoryPoint`
- RViz 可视化：
  - 实际轨迹
  - 期望轨迹
  - 实际/期望姿态轴
  - 位置误差箭头
  - 相机跟随机体 TF
- rosbag 录制与离线分析：
  - 自动录制 `/quad_state`、`/trajectory`、`/control_input`
  - 离线绘制 3D 轨迹图、误差图
  - 输出 `summary.json`
- 一键实验脚本：
  - Case I：椭圆螺旋跟踪
  - Case II：接近倒置姿态恢复

## Workspace Layout

```text
ros2_p_ws/
├── README.md
├── scripts/
│   ├── analyze_bag.py
│   ├── record_bag.sh
│   ├── run_case1.sh
│   └── run_case2.sh
└── src/
    ├── quad_se3_msgs/
    │   └── msg/
    │       ├── ControlInput.msg
    │       ├── QuadState.msg
    │       └── TrajectoryPoint.msg
    └── quad_se3_py/
        ├── launch/
        │   ├── sim.launch.py
        │   ├── playback_viz.launch.py
        │   └── sim_viz.launch.py
        ├── quad_se3_py/
        │   ├── analysis_timebase.py
        │   ├── config.py
        │   ├── reference.py
        │   ├── sim_node.py
        │   ├── trajectories.py
        │   ├── utils.py
        │   └── visualization_node.py
        ├── rviz/
        │   ├── quad_recording.rviz
        │   └── quad_se3.rviz
        └── test/
```

## Environment

建议环境：

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10

如果你还没有 source ROS 2：

```bash
source /opt/ros/humble/setup.bash
```

## Build

在工作区根目录执行：

```bash
cd /home/sachan/ros2_p_ws
colcon build
source install/setup.bash
```

如果只想构建本项目包：

```bash
colcon build --packages-select quad_se3_msgs quad_se3_py
source install/setup.bash
```

## Main Topics

核心话题如下：

- `/trajectory`
  - 类型：`quad_se3_msgs/msg/TrajectoryPoint`
  - 来源：`sim_node`
  - 用途：与当前 `/quad_state` 同 stamp 的参考轨迹观测流，便于录包、调试和离线核对
- `/control_input`
  - 类型：`quad_se3_msgs/msg/ControlInput`
  - 来源：`sim_node`
- `/quad_state`
  - 类型：`quad_se3_msgs/msg/QuadState`
  - 来源：`sim_node`

RViz 可视化相关话题：

- `/viz/actual_path`
- `/viz/desired_path`
- `/viz/actual_pose`
- `/viz/desired_pose`
- `/viz/markers`

相机跟随使用的 TF：

- `world -> quad_actual`

时间对齐约定：

- `sim_node` 在一个固定 `0.002 s` 主循环里统一推进仿真时间
- `/quad_state.stamp` 与 `/trajectory.stamp` 在同一步内完全一致
- `/trajectory_epoch` 记录共享轨迹时间零点，供可视化和离线分析重建参考
- `visualization_node` 继续以 `QuadState.stamp` 为准重算期望轨迹

## Launch Files

### 1. 纯仿真

```bash
ros2 launch quad_se3_py sim.launch.py
```

### 2. 仿真 + RViz

```bash
ros2 launch quad_se3_py sim_viz.launch.py
```

常用参数：

```bash
ros2 launch quad_se3_py sim_viz.launch.py \
  use_rviz:=true \
  use_sim_time:=false \
  trajectory_mode:=paper_case_1_helix \
  trajectory_start_time_sec:=<shared_start_time> \
  path_max_points:=2000 \
  show_error_markers:=true
```

如果不显式传 `trajectory_start_time_sec`，`sim_node` 会在启动时自动生成共享 epoch。

也支持显式指定 RViz 配置：

```bash
ros2 launch quad_se3_py sim_viz.launch.py \
  rviz_config:=/home/sachan/ros2_p_ws/install/quad_se3_py/share/quad_se3_py/rviz/quad_recording.rviz
```

支持的初始姿态参数：

```bash
initial_roll_deg:=0.0
initial_pitch_deg:=0.0
initial_yaw_deg:=0.0
```

## Trajectory Modes

`sim_node` 当前支持这些模式：

- `hover`：用于基础闭环验证
- `paper_case_1_helix`：对应论文风格的空间轨迹跟踪演示
- `paper_case_2_recovery_reference`：用于倒置恢复场景，重点在初始姿态设置

轨迹查询规则：

- `evaluate_trajectory(mode, t_sec)` 是统一参考入口
- `sim_node` 会在积分后按发布 stamp 再采样一份 `/trajectory`
- `visualization_node` 会按 `state.stamp` 查询
  `evaluate_trajectory(mode, t_state - trajectory_start_time_sec)`
- 当前默认不做显式延迟补偿；未来若需要，可通过
  `reference_time_offset_sec` 引入固定时间偏移

## Recommended Runs

### 基础可视化检查

```bash
cd /home/sachan/ros2_p_ws
source install/setup.bash
ros2 launch quad_se3_py sim_viz.launch.py trajectory_mode:=hover
```

### Case I: 椭圆螺旋轨迹

```bash
cd /home/sachan/ros2_p_ws
./scripts/run_case1.sh
```

- 脚本会先启动 rosbag recorder，再启动仿真，尽量避免漏掉开头数据
- 默认使用更干净的录制视图：关闭 TF，隐藏位置误差箭头
- 停止后会自动分析最新 bag
- 分析结果默认写到带时间戳的目录，不会覆盖上一次结果

如果不想启动 RViz：

```bash
USE_RVIZ=false ./scripts/run_case1.sh
```

如果想显示位置误差箭头：

```bash
SHOW_ERROR_MARKERS=true ./scripts/run_case1.sh
```

### Case II: 接近倒置姿态恢复

```bash
cd /home/sachan/ros2_p_ws
./scripts/run_case2.sh
```

- 脚本同样会先启动 recorder，再启动仿真
- 默认也使用录制友好的 RViz 视图
- 停止后自动分析最新 bag
- 分析结果默认按 bag 目录名输出

默认初始姿态是：

```text
roll = 178 deg
pitch = 0 deg
yaw = 0 deg
```

也可以这样覆盖：

```bash
INITIAL_ROLL_DEG=175 USE_RVIZ=false ./scripts/run_case2.sh
```

## Video Recording Workflow

推荐用于论文风格录屏：

```bash
cd /home/sachan/ros2_p_ws
source install/setup.bash
./scripts/run_case1.sh
```

录制建议：

- 启动后把 RViz 窗口最大化
- 保持 `quad_recording.rviz` 的单一主镜头，不要边录边转动视角
- 如果画面还想更干净，继续保持 `TF` 关闭
- 如果想强调跟踪误差，再临时打开 `SHOW_ERROR_MARKERS=true`

当前默认录制配置：

- 保留 `Actual Path`
- 保留 `Desired Path`
- 保留实际/期望姿态轴 Marker
- 默认关闭 `TF`
- 默认关闭位置误差箭头

录制视图文件位于：

[`src/quad_se3_py/rviz/quad_recording.rviz`](/home/sachan/ros2_p_ws/src/quad_se3_py/rviz/quad_recording.rviz)

## rosbag Recording

手动录包：

```bash
cd /home/sachan/ros2_p_ws
./scripts/record_bag.sh case1_helix
```

录制内容：

- `/quad_state`
- `/trajectory_epoch`
- `/trajectory`
- `/control_input`

录制结果会保存到：

```text
bags/<case_name>_<timestamp>/
```

例如：

```text
bags/case1_helix_20260328_204828/
```

## Offline Analysis

### 分析最新 bag

```bash
cd /home/sachan/ros2_p_ws
python3 scripts/analyze_bag.py
```

- 自动选择 `bags/` 下最新的一个 bag
- 输出目录默认使用该 bag 的目录名，因此会保留时间戳
- 不会覆盖之前的分析结果

### 分析指定 bag

```bash
python3 scripts/analyze_bag.py bags/case1_helix_20260328_204828
```

离线分析的参考时间零点优先级是：

- `--trajectory-start-time-sec`
- bag 中录到的 `/trajectory_epoch`
- `experiment_metadata.json` 中的 `trajectory_start_time_sec`
- 首条 `/trajectory.stamp`（仅作为最后回退）

参考时间偏移优先级是：

- `--reference-time-offset-sec`
- `experiment_metadata.json` 中的 `reference_time_offset_sec`
- 默认 `0.0`

这样可以让实时仿真、慢放回放和离线误差图尽量使用同一套时间口径。

### 指定输出目录

```bash
python3 scripts/analyze_bag.py bags/case1_helix_20260328_204828 \
  --output-dir plots/manual_case1
```

也支持显式覆盖参考时间偏移：

```bash
python3 scripts/analyze_bag.py bags/case1_helix_20260328_204828 \
  --reference-time-offset-sec 0.02
```

分析输出默认写到：

```text
plots/<bag_directory_name>/
```

例如：

```text
plots/case1_helix_20260328_204828/
```

包含：

- `trajectory_3d.png`
- `errors.png`
- `summary.json`

## Slow Playback

如果实时录制还是太快，可以回放最近一个 bag 并慢速录屏：

```bash
cd /home/sachan/ros2_p_ws
./scripts/replay_bag_slow.sh
```

默认行为：

- 自动选择 `bags/` 下最新的一个 bag
- 启动仅用于回放的可视化 launch
- 使用 `/clock` 驱动 RViz 和 visualization 节点
- 默认以 `0.5x` 速度播放
- 优先使用 bag 中记录的 `/trajectory_epoch` 和 metadata 里的
  `reference_time_offset_sec` 来对齐参考时间

也可以指定 bag 和倍率：

```bash
PLAY_RATE=0.4 ./scripts/replay_bag_slow.sh bags/case1_helix_20260328_204828
```

如果想在慢放时保留误差箭头：

```bash
PLAY_RATE=0.6 SHOW_ERROR_MARKERS=true ./scripts/replay_bag_slow.sh
```

## What the Analysis Computes

离线分析目前会计算这些量：

- 位置误差 `e_x = x - x_d`
- 速度误差 `e_v = v - v_d`
- 姿态误差向量 `e_R`
- 角速度误差 `e_Omega`
- 姿态误差函数 `Psi`

并绘制：

- 实际/期望 3D 轨迹
- `||e_x||`
- `Psi`
- `||e_Omega||`
- 推力曲线

## Testing

运行 Python 包测试：

```bash
cd /home/sachan/ros2_p_ws
colcon test --packages-select quad_se3_py --event-handlers console_direct+
```

查看测试结果：

```bash
colcon test-result --all
```

## Common Issues

### 1. `quad_se3_msgs` not found

症状：

```text
Topic '/control_input' has unknown type 'quad_se3_msgs/msg/ControlInput'
```

原因：

- 没有 source 当前工作区
- 或者脚本运行时继承了别的 ROS 工作区环境

解决：

```bash
cd /home/sachan/ros2_p_ws
colcon build
source install/setup.bash
```

项目中的 `record_bag.sh` 已经会自动加载当前工作区环境。

### 2. 分析结果被覆盖

现在默认不会覆盖。

`analyze_bag.py` 会把输出写到：

```text
plots/<bag_directory_name>/
```

也就是说，只要 bag 目录带时间戳，分析结果也会自动带时间戳。

只有在你显式传入 `--output-dir` 时，才会写到你指定的固定目录。

如果误差图看起来和实时 RViz 不一致，先检查 bag 里是否录到了
`/trajectory_epoch`，以及 `experiment_metadata.json` 中的
`trajectory_epoch_source` 是否为 `trajectory_epoch_topic`。

### 3. VS Code / Pylance 无法解析 `quad_se3_py`

如果编辑器提示导入错误，重载 VS Code 窗口即可：

```text
Ctrl+Shift+P -> Developer: Reload Window
```

### 4. RViz 镜头不跟随

检查 `Views` 面板：

- `Type` 为 `Orbit`
- `Target Frame` 为 `quad_actual`

### 5. 画面里坐标架太多

RViz 里当前可能同时显示：

- TF
- Marker 里的实际姿态轴
- Marker 里的期望姿态轴

如果想让画面更干净，可以在 RViz 的 `Markers` 或 `TF` 显示里手动关掉一部分。

录制场景下，项目现在默认使用 `quad_recording.rviz`，已经把 `TF` 关闭，优先保留路径和姿态轴。

## Development Notes

当前实现更偏“闭环仿真 + 可视化 + 离线评估”阶段，适合：

- 调控制参数
- 验证姿态/轨迹跟踪行为
- 复现论文中的典型演示场景

当前默认运行链路已经切换到 `sim_node`。旧的 `controller_node.py`、
`dynamics_node.py`、`trajectory_node.py` 仍保留在仓库中作为参考实现，
但不再由默认 launch 启动。

控制增益入口现在统一放在
[`src/quad_se3_py/quad_se3_py/config.py`](/home/sachan/ros2_p_ws/src/quad_se3_py/quad_se3_py/config.py)，
其中 `make_control_gains()` 统一生成 `kx`、`kv`、`kR`、`kOmega`，在线仿真、
可视化和离线分析共用这一份默认配置。

还没有做的事情包括：

- 更完整的论文参数对齐与系统辨识
- 更严格的数值积分与仿真稳定性分析
- 更细的误差图配置
- 更完整的文档和实验结果对照

## Quick Start

如果你只想最快看到结果：

```bash
cd /home/sachan/ros2_p_ws
colcon build --packages-select quad_se3_msgs quad_se3_py
source install/setup.bash
./scripts/run_case1.sh
```

停止后自动分析。随后查看：

```text
plots/case1_helix_<timestamp>/
```
