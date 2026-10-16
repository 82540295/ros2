"""AEB（自动紧急制动）决策核心。

纯离线逻辑，只依赖 numpy，不依赖 ROS / CARLA，可完整单元测试：

1. 深度图 → 前方障碍物距离（decode_carla_depth / depth_min_distance）
2. AEB 状态机（aeb_decide / AEBController）：
   cruise → warning → braking → recover，每个状态都有进入/退出
   滞回阈值，避免临界抖动。
"""
from dataclasses import dataclass

import numpy as np

# 状态机状态
CRUISE = "cruise"      # 巡航
WARNING = "warning"    # 预警（障碍物渐近，部分制动）
BRAKING = "braking"    # 紧急制动（全力刹车）
RECOVER = "recover"    # 恢复（障碍消除后缓释）
ALL_STATES = (CRUISE, WARNING, BRAKING, RECOVER)


@dataclass
class AEBParams:
    """AEB 决策参数（距离单位米，速度单位 m/s）。"""
    warn_enter: float = 18.0     # 进入预警：前方距离低于此值
    warn_exit: float = 24.0      # 退出预警：前方距离高于此值（滞回）
    brake_enter: float = 7.0     # 进入紧急制动：前方距离低于此值且正在接近
    brake_exit: float = 12.0     # 退出紧急制动：前方距离高于此值（滞回）
    close_min: float = 0.5       # 判定“正在接近”的最小相对接近速度
    cruise_throttle: float = 0.55
    warn_brake: float = 0.15     # 预警时的温和预制动（过强会把接近速度拖垮，
                                  # 导致 d 到达 brake_enter 时已不满足“正在接近”）
    brake_max: float = 1.0       # 全力制动
    valid_min: float = 0.3       # 深度有效下限（滤掉车身自身）
    valid_max: float = 80.0      # 深度有效上限（超过视为无障碍）


def decode_carla_depth(raw, width, height):
    """把 CARLA sensor.camera.depth 的原始字节流解码为 (H, W) float 深度图（米）。

    CARLA 相机原始数据按 BGRA 打包（H, W, 4）。深度编码为 16 位整数
    （精度 1/256 米，量程 ~256 m）：红通道（缓冲第 3 字节）= 高字节，
    绿色通道（缓冲第 2 字节）= 低字节。
    """
    buf = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 4)
    r = buf[:, :, 2].astype(np.float32)   # 红 = 高字节
    g = buf[:, :, 1].astype(np.float32)   # 绿 = 低字节
    return (r * 256.0 + g) / 256.0


def encode_carla_depth(depth):
    """decode_carla_depth 的逆操作（测试与演示用）：米 → BGRA 原始字节流。"""
    d = np.clip(np.asarray(depth, dtype=np.float32), 0.0, 65535.0 / 256.0)
    v = np.round(d * 256.0).astype(np.uint16)
    out = np.zeros((depth.shape[0], depth.shape[1], 4), dtype=np.uint8)
    out[:, :, 2] = (v >> 8).astype(np.uint8)   # 红 = 高字节
    out[:, :, 1] = (v & 0xFF).astype(np.uint8)  # 绿 = 低字节
    out[:, :, 3] = 255                          # A
    return out.tobytes()


def depth_min_distance(depth, row0=0.30, row1=0.85, col0=0.20, col1=0.80,
                       valid_min=0.3, valid_max=80.0):
    """深度图中央前向区域内最小有效距离（米）。

    row0/row1、col0/col1 为 0~1 的归一化坐标，圈出“正前方”区域
    （避开天空、地面远端与画面两侧）。区域内无有效障碍时返回 valid_max。
    """
    h, w = depth.shape
    r0, r1 = int(h * row0), max(int(h * row1), int(h * row0) + 1)
    c0, c1 = int(w * col0), max(int(w * col1), int(w * col0) + 1)
    band = depth[r0:r1, c0:c1]
    band = band[(band >= valid_min) & (band <= valid_max)]
    if band.size == 0:
        return float(valid_max)
    return float(band.min())


def frame_min_distance(raw, width, height, valid_min=0.3, valid_max=80.0):
    """一帧深度相机原始数据 → 中央前向区域最小有效距离（米）。

    桥接节点每帧调用；纯函数，便于离线单测。
    """
    depth = decode_carla_depth(raw, width, height)
    return depth_min_distance(depth, valid_min=valid_min, valid_max=valid_max)


def estimate_closing(d_prev, d_now, dt, fallback=0.0):
    """由前后两帧前向距离估计相对接近速度（m/s，>0 表示正在接近）。

    首帧（d_prev 为 None）或间隔非法（dt <= 0）时返回 fallback
    （调用方可传自车速度作为静止障碍物的近似）。
    """
    if d_prev is None or dt <= 0.0:
        return float(fallback)
    return (d_prev - d_now) / dt


def aeb_decide(state, d, closing, p=None):
    """AEB 状态机单步决策。

    参数：
        state:   当前状态（ALL_STATES 之一）
        d:       前方障碍物距离（米；无障碍时为 valid_max）
        closing: 相对接近速度（m/s，>0 表示正在接近）
        p:       AEBParams

    返回 (new_state, throttle, brake, event)；
    event 为 "warn" / "brake" / "recover" / None。
    """
    if p is None:
        p = AEBParams()
    if state not in ALL_STATES:
        raise ValueError(f"unknown AEB state: {state!r}")
    event = None

    # 进入紧急制动（最高优先级）：足够近且正在接近
    if state != BRAKING and d < p.brake_enter and closing > p.close_min:
        state, event = BRAKING, "brake"
    elif state == BRAKING and d > p.brake_exit:
        state, event = RECOVER, "recover"

    if state == RECOVER:
        if d > p.warn_exit:
            state, event = CRUISE, None
    elif state == WARNING:
        if d > p.warn_exit:
            state, event = CRUISE, None
    elif state == CRUISE:
        if d < p.warn_enter:
            state, event = WARNING, "warn"

    if state == BRAKING:
        return state, 0.0, p.brake_max, event
    if state == WARNING:
        return state, p.cruise_throttle * 0.5, p.warn_brake, event
    if state == RECOVER:
        return state, 0.25, 0.25, event
    return state, p.cruise_throttle, 0.0, event


class AEBController:
    """有状态的 AEB 控制器：每 tick 喂入 (距离, 接近速度)，输出 (油门, 制动)。"""

    def __init__(self, params=None, state=CRUISE):
        self.p = params or AEBParams()
        self.state = state
        self.history = []   # (t, state, d, closing, throttle, brake, event)
        self.events = []    # (t, event)

    def tick(self, d, closing, t=None):
        new_state, throttle, brake, event = aeb_decide(self.state, d, closing, self.p)
        self.state = new_state
        self.history.append((t, self.state, d, closing, throttle, brake, event))
        if event is not None:
            self.events.append((t, event))
        return self.state, throttle, brake, event

    def abort(self):
        """紧急中止：立即回到 cruise 并释放制动（障碍仍近时下一 tick 会重新预警）。"""
        self.state = CRUISE

    def reset(self):
        """复位状态机与历史记录。"""
        self.state = CRUISE
        self.history.clear()
        self.events.clear()
