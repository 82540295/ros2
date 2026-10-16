"""CARLA 0.9.16 连接与车辆/传感器工具函数（AEB 模块版）。

沿用 carla_perception_control 的已验证惯例：
  * carla 惰性导入（未安装时给出友好提示，离线测试不依赖 carla）
  * 同步模式 + 固定步长，逐帧精确控制
  * 退出前必须调用 restore_async()，否则服务端永久停帧
"""
import os

import numpy as np

try:
    import carla
except ImportError:  # 未装 carla 库时给出友好提示
    carla = None

DEFAULT_HOST = os.environ.get("CARLA_HOST", "127.0.0.1")
DEFAULT_PORT = int(os.environ.get("CARLA_PORT", "2000"))
# AEB 演示需要较长直道路段；Town01_Opt 主路足够直且轻量
DEFAULT_MAP = os.environ.get("CARLA_MAP", "Town01_Opt")
EGO_BLUEPRINT = os.environ.get("CARLA_EGO_BP", "vehicle.tesla.model3")

DT = 0.05  # 同步模式固定步长（秒），20 Hz


def _check_carla():
    if carla is None:
        raise RuntimeError(
            "未找到 carla 模块。请安装 CARLA 0.9.16 的 Python 客户端："
            "pip3 install <CARLA>/PythonAPI/carla/dist/carla-0.9.16-cp310-*.whl"
        )


def connect(host=DEFAULT_HOST, port=DEFAULT_PORT, town=DEFAULT_MAP,
            sync=True, dt=DT, timeout=60.0, force_reload=False):
    """连接 CARLA 服务端并加载地图，返回 (client, world)。

    * 服务端已加载目标地图时跳过重载（load_world 是重操作，省数秒）
    * 开启同步模式（synchronous_mode）+ 固定步长，便于逐帧控制
    """
    _check_carla()
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    try:
        current = client.get_world().get_map().name
    except Exception:  # noqa: BLE001
        current = None

    # 地图名比较：服务端返回形如 "Carla/Maps/Town01_Opt"，也接受 "Town01_Opt"
    def _same(name, want):
        return bool(name) and (name == want or name.endswith("/" + want))

    if current and _same(current, town) and not force_reload:
        world = client.get_world()
        print(f"[连接] 服务端已在地图 {current}，跳过重载")
    else:
        print(f"[连接] 正在加载地图 {town}（当前 {current or '未知'}），可能需要数秒...")
        world = client.load_world(town)
        print(f"[连接] 地图 {town} 加载完成")
    if sync:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)
    return client, world


def restore_async(world):
    """把世界恢复为异步模式（节点/脚本退出前必须调用）。

    同步模式下服务端只在客户端 world.tick() 时推进；退出后无人 tick，
    服务端就永久停帧——CARLA 窗口“卡住”，后来者连接也像死机。
    """
    _check_carla()
    try:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        return True
    except Exception:  # noqa: BLE001
        return False


def spawn_vehicle(world, blueprint=EGO_BLUEPRINT, transform=None):
    """按蓝图生成自车，返回 (vehicle, transform)。

    transform 为 None 时取地图第一个出生点；该点生成失败时
    回退到最近可行走路点的变换。
    """
    _check_carla()
    bp = world.get_blueprint_library().find(blueprint)
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "hero")
    if transform is None:
        transform = world.get_map().get_spawn_points()[0]
    vehicle = world.try_spawn_actor(bp, transform)
    if vehicle is None:
        wp = world.get_map().get_waypoint(transform.location)
        tf = wp.transform
        tf.location.z += 0.6
        vehicle = world.try_spawn_actor(bp, tf)
    if vehicle is None:
        raise RuntimeError("无法生成自车，请检查 CARLA 服务端与地图。")
    return vehicle, transform


def make_depth_camera(world, vehicle, callback, width=640, height=360, fov=90,
                      x=1.8, z=1.3, tick=False):
    """在车辆前部挂深度相机，返回 sensor。

    callback 收到 carla.Image：raw_data 为 BGRA 字节流（H*W*4），
    解码见 aeb_core.decode_carla_depth / frame_min_distance。
    """
    _check_carla()
    bp = world.get_blueprint_library().find("sensor.camera.depth")
    bp.set_attribute("image_size_x", str(width))
    bp.set_attribute("image_size_y", str(height))
    bp.set_attribute("fov", str(fov))
    tf = carla.Transform(carla.Location(x=x, z=z))
    sensor = world.spawn_actor(bp, tf, attach_to=vehicle)
    if tick:
        world.tick()
    sensor.listen(callback)
    return sensor


def apply_control(vehicle, throttle=0.0, steer=0.0, brake=0.0, reverse=False):
    """给车辆施加（油门, 转向, 刹车）。throttle/brake ∈ [0,1]，steer ∈ [-1,1]。"""
    vc = carla.VehicleControl(
        throttle=float(np.clip(throttle, 0.0, 1.0)),
        steer=float(np.clip(steer, -1.0, 1.0)),
        brake=float(np.clip(brake, 0.0, 1.0)),
        reverse=bool(reverse),
        hand_brake=False,
    )
    vehicle.apply_control(vc)
    return vc


def get_speed(vehicle):
    """车辆速度（m/s，水平分量模）。"""
    v = vehicle.get_velocity()
    return float(np.hypot(v.x, v.y))


def get_location(vehicle):
    """返回 (x, y, z)。"""
    loc = vehicle.get_location()
    return (loc.x, loc.y, loc.z)


def get_yaw(vehicle):
    """航向角（弧度，CARLA 约定）。"""
    import math
    return math.radians(vehicle.get_transform().rotation.yaw)
