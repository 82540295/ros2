# 无人机航点任务飞行（uav_waypoint_mission）

本模块在 **AirSim / CarlaAir** 仿真器上，使用 **ROS 2 Humble** 实现无人机**起飞 → 按顺序飞越航点 → 垂直降落**的完整自主任务流程。任务状态机核心是纯 Python 的确定性实现（不依赖 ROS 与仿真器），任何机器上都能跑离线单元测试。

## 模块特点

- **可离线验证**：状态机核心 `waypoint/mission.py` 不 import ROS / AirSim，`python3 main.py test` 在裸 Python 环境即可验证全链路（正常任务、航点超时、急停降落、配置加载）
- **航线可配置**：航点写在 `config/mission.yaml`（ENU 坐标，起飞点为原点），改文件即可改航线，默认是一条 Town01 方形航线（4 个航点）
- **自带动仿真配置**：`config/airsim_settings.json` 定义 WaypointDrone（SimpleFlight 飞控）+ 前视 RGB 相机 + 16 线激光雷达 + IMU，拷到 `~/Documents/AirSim/settings.json` 即可用
- **标准 ROS 2 包**：ament_python 功能包，`ros2 launch uav_waypoint_mission main.launch.py` 启动，参数 `host / vehicle / mission / rviz`

## 环境要求

| 项目 | 要求 |
|---|---|
| 系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble（`sudo apt install ros-humble-desktop`） |
| Python | 3.10（Humble 系统 Python）；`pyyaml` 随 ROS 自带 |
| 仿真器（在线运行） | CarlaAir（OpenHUTB 发行版）或任意 AirSim 场景，RPC 端口 41451 |

## 安装与构建

```bash
# 进入 ros2 工作区
cd <ros2 工作区根目录>
source install/setup.bash 2>/dev/null || true

# 模块自检 + 离线测试 + 构建（也可分步执行）
cd src/air/uav_waypoint_mission
python3 main.py doctor
python3 main.py test
python3 main.py build
```

预期测试输出：

```
Ran 15 tests in 0.006s

OK
```

## 在线运行：状态发布节点

`uav_state_source` 节点通过 AirSim RPC（端口 41451）连接仿真器，**启动时执行 ping 自检**（连不上会立即抛错退出），之后以 10 Hz 发布车辆状态：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/uav/odometry` | `nav_msgs/Odometry` | ENU map 坐标系、FLU 车体坐标系的位置/姿态/速度 |
| `/uav/state` | `std_msgs/String` | 版本化 JSON（schema_version=1，NED/FRD 原始值） |

前置准备（一次性）：

1. 把载具配置拷入仿真器：`cp config/airsim_settings.json ~/Documents/AirSim/settings.json`
2. 启动 CarlaAir：`./CarlaUE4.sh /Game/Carla/Maps/Town01 -quality-Low`，等到日志出现 `Ready! Both servers are running.`

启动节点并验证：

```bash
python3 main.py live --host <仿真器所在机器IP>
# 预期日志：state source connected to <host> (vehicle WaypointDrone)

# 另开终端（已 source ROS 2）：
ros2 topic hz /uav/odometry
ros2 topic echo /uav/odometry --once
```

坐标换算（NED↔ENU、FRD↔FLU）为纯函数，见 `waypoint/coords.py`，离线单元测试覆盖 yaw 0°/90° 速度转换、下降速度符号等场景。

## 航点配置

`config/mission.yaml` 中所有位置为 **ENU 坐标**（x 东、y 北、z 上，单位米），相对无人机起飞点（仿真器出生点）。默认航线：

```yaml
mission:
  name: town01_square
  takeoff_altitude_m: 3.0     # 起飞悬停高度
  cruise_speed_mps: 2.0       # 巡航速度
  land_speed_mps: 0.5         # 降落速度
  waypoint_timeout_s: 60.0    # 单个航点超时（秒），超时任务判失败
  waypoints:
    - {name: wp1_east,  x: 10.0, y: 0.0,  z: 3.0}
    - {name: wp2_north, x: 10.0, y: 10.0, z: 4.0}
    - {name: wp3_west,  x: 0.0,  y: 10.0, z: 3.0}
    - {name: wp4_home,  x: 0.0,  y: 0.0,  z: 3.0}
```

## 飞行状态机

```
IDLE --arm()--> ARMED --go()--> TAKING_OFF --到达悬停高度--> WAYPOINT(wp1..wpN)
   --全部到达--> LANDING --z<=0.3m--> LANDED
```

- 到达判据：水平距离 < 0.5 m 且高度差 < 0.3 m
- 任一航点超时（> `waypoint_timeout_s`）→ `FAILED`（停止并收桨）
- 飞行中（TAKING_OFF / WAYPOINT）随时可 `abort()`：从当前位置垂直降落
- 与 AirSim 的 NED 坐标系换算在 ROS2 适配层完成（后续提交）

## 验证（当前提交）

离线单元测试共 15 个，全部通过：

| 测试 | 验证点 |
|---|---|
| test_repo_config_loads | 仓库默认航线配置可加载、字段正确 |
| test_empty_waypoints_rejected | 空航点列表被拒绝 |
| test_full_mission_sequence | 完整链路 IDLE→…→LANDED，航点按序访问 |
| test_go_requires_armed | 未解锁不能起飞（安全约束） |
| test_takeoff_command_targets_home_altitude | 起飞指令目标 = 原点 + 悬停高度 |
| test_waypoint_timeout_fails | 航点超时 → FAILED 且指令为停止 |
| test_abort_lands_vertically | 急停 → 从当前位置垂直降落 → LANDED |
| test_coords（5 个） | NED↔ENU 点位换算与往返、yaw 0°/90° 速度转换、下降速度符号、角速度符号翻转、姿态四元数归一 |

在线仿真验证（起飞/飞航点/降落的窗口截图与动图）在后续提交补齐。

## 相关模块

- [建立虚拟机和空域载具之间的连接](./setup_and_connect.md)
- [无人机与 ROS 的桥接模块](./carlair_ros_bridge.md)
- [基于 GRU 的无人机飞行状态预测与 ROS2 在线误差分析](./state_prediction.md)
- [无人机键盘控制](./uav_keyboard_control.md)
