# uav_waypoint_mission 无人机航点任务飞行

基于 **AirSim / CarlaAir** 仿真器与 **ROS 2 Humble**，实现多旋翼无人机「起飞 → 按顺序飞越航点 → 垂直降落」的完整任务。任务状态机核心为纯 Python（无 ROS / 仿真器依赖），可在任意机器上离线单元测试。

## 功能

- `waypoint/mission.py` 任务状态机核心：确定性状态机，纯 Python，`python3 main.py test` 离线验证
- `config/mission.yaml` 航点定义（ENU 坐标，以起飞点为原点），改这一个文件即可改航线
- `config/airsim_settings.json` 仿真器载具配置：WaypointDrone（SimpleFlight）+ 前视 RGB 相机 + 16 线激光雷达 + IMU
- ROS 2 Humble `ament_python` 功能包，`ros2 launch uav_waypoint_mission main.launch.py` 启动
- **状态发布节点** `uav_state_source`：连接仿真器 RPC 41451（启动 ping 自检，连不上立即报错），10 Hz 发布 ENU 里程计

## 目录结构

```
uav_waypoint_mission/
├── main.py / main.sh         # 统一入口：doctor / test / build / demo / live
├── config/
│   ├── mission.yaml          # 航点任务定义（ENU，相对起飞点）
│   └── airsim_settings.json  # AirSim 载具与传感器配置
├── launch/main.launch.py     # ros2 launch 参数：host / vehicle / mission / rviz
├── waypoint/
│   ├── mission.py            # 任务模型 + 确定性状态机（纯 Python）
│   ├── coords.py             # NED/ENU、FRD/FLU 坐标换算（纯 Python）
│   └── ros_nodes.py          # ROS2 节点（状态发布）
└── tests/
    ├── test_mission.py       # 状态机离线单元测试（不需要 ROS / 仿真器）
    └── test_coords.py        # 坐标换算离线单元测试
```

## 环境要求

| 项目 | 要求 |
|---|---|
| 系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble（`sudo apt install ros-humble-desktop`） |
| Python | 3.10（Humble 系统 Python）；`pyyaml`（随 ROS 自带） |
| 仿真器（demo/live 阶段） | CarlaAir（OpenHUTB 发行版）或任意 AirSim 场景，RPC 端口 41451 |

## 快速开始

```bash
cd src/air/uav_waypoint_mission

# 1. 环境自检（Python 版本 + pyyaml）
python3 main.py doctor

# 2. 离线单元测试（状态机全链路 / 航点超时 / 急停降落 / 配置加载，不需要 ROS 与仿真器）
python3 main.py test

# 3. 构建 ROS2 功能包（需已 source ROS 2 Humble 的终端）
python3 main.py build

# 4. 连接仿真器发布状态（需仿真器已启动，且 config/airsim_settings.json 已拷到 ~/Documents/AirSim/）
python3 main.py live --host 192.168.122.1
# 另开终端验证：
ros2 topic echo /uav/odometry --once
```

> `demo` 阶段与任务控制 / 键盘遥控节点在后续提交中陆续加入。

## 飞行逻辑（状态机）

```
IDLE --arm()--> ARMED --go()--> TAKING_OFF --到达悬停高度--> WAYPOINT(wp1..wpN)
   --全部到达--> LANDING --z<=0.3m--> LANDED
```

- 任一航点超时（> `waypoint_timeout_s`）→ `FAILED`（停止并收桨）
- 飞行中（TAKING_OFF / WAYPOINT）随时可 `abort()`，从当前位置垂直降落
- 坐标统一 ENU（x 东、y 北、z 上）；与 AirSim NED 的换算在 ROS2 适配层完成（后续提交）

## 后续计划（本系列提交）

- [x] AirSim 状态发布节点：连接 41451，发布 `/uav/odometry`、`/uav/state`（提交 2）
- [ ] 任务控制节点：状态机 + `moveOnPositionAsync` 驱动仿真器（提交 3）
- [ ] 键盘遥控节点：WASD/QE/RF，任务暂停/恢复（提交 4）
- [ ] 文档完善与仿真演示（提交 5）
