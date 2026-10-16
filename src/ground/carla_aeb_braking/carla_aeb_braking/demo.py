"""AEB 一键演示。

用法：
    python3 main.py demo            # 离线：制动曲线仿真（不需要 ROS / CARLA）
    python3 main.py demo --live     # 真实：CARLA 场景（生成障碍车 → 触发 AEB → 截图 + 数据导出）

输出到 artifacts/（本目录已 gitignore）：
    离线   aeb_offline_curve.png / aeb_offline_data.csv
    --live aeb_live_curve.png / aeb_live_data.csv / aeb_live_brake.png

离线场景：自车 43 km/h 巡航逼近前方以 18 km/h 行驶的车辆，
AEB 依次经历 预警(18m) → 全力制动(7m) → 刹停 → 恢复 → 重新巡航
（完整状态链 + 恢复过程，滞回阈值防抖）。
"""
import csv
import math
from pathlib import Path

ARTIFACTS = Path(__file__).resolve().parent.parent / "artifacts"

# ---- 离线纵动力学玩具模型（仅用于演示与自检，非精确车辆模型）----
THRU_ACCEL = 4.0    # 油门满行程加速度 (m/s^2)
BRAKE_DECEL = 8.6   # 制动满行程减速度 (m/s^2，~0.9g，真实 AEB 量级)
DRAG = 0.18         # 线性气动/滚动阻力系数（终端巡航速 ≈ 油门*4/DRAG ≈ 12.2 m/s）

STATE_ORDER = {"cruise": 0, "warning": 1, "braking": 2, "recover": 3}


def simulate_aeb_scenario(v0=12.0, d0=60.0, v_obs=5.0, dt=0.05, t_max=90.0, p=None):
    """离线纵动力学仿真：自车以 v0 巡航，前方车辆以 v_obs 同向行驶，AEB 决策。

    参数：
        v0:    自车初速 (m/s，默认 12 ≈ 43 km/h)
        d0:    初始前向距离 (m)
        v_obs: 障碍车速度 (m/s，默认 5 ≈ 18 km/h)
        dt:    步长 (s)
        p:     AEBParams

    返回 (history, meta)：
        history: [(t, state, d, v, closing, throttle, brake, event), ...]
        meta:    {"min_gap", "brake_time", "stopped", "final_state", "t_end"}
    """
    from carla_aeb_braking.aeb_core import AEBController, AEBParams
    p = p or AEBParams()
    ctl = AEBController(p)
    d, v = float(d0), float(v0)
    t = 0.0
    history = []
    min_gap = float("inf")
    brake_time = None
    stopped = False
    while t < t_max:
        closing = max(v - v_obs, 0.0)
        state, throttle, brake, event = ctl.tick(d, closing, t=t)
        accel = throttle * THRU_ACCEL - brake * BRAKE_DECEL - DRAG * v
        v = max(0.0, v + accel * dt)
        d = max(0.0, d - (v - v_obs) * dt)
        min_gap = min(min_gap, d)
        if event == "brake" and brake_time is None:
            brake_time = t
        history.append((t, state, d, v, closing, throttle, brake, event))
        t += dt
        if v < 0.05 and d < p.brake_exit:
            stopped = True
        # 一次完整 AEB 事件结束：全力刹过车、已刹停、且恢复巡航
        if brake_time is not None and stopped and state == "cruise":
            break
    meta = {
        "min_gap": min_gap,
        "brake_time": brake_time,
        "stopped": stopped,
        "final_state": ctl.state,
        "t_end": t,
    }
    return history, meta


def _write_csv(rows, path, fields):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(fields)
        w.writerows(rows)


def plot_curve(history, meta, out_png, title="AEB 离线演示"):
    """三行图：前向距离 / 车速 / 制动与状态。返回 True 表示成功。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[demo] 未安装 matplotlib，跳过绘图（pip install matplotlib）")
        return False

    t = [h[0] for h in history]
    d = [h[2] for h in history]
    v = [h[3] for h in history]
    brake = [h[6] for h in history]
    state = [STATE_ORDER.get(h[1], 0) for h in history]
    from carla_aeb_braking.aeb_core import AEBParams
    pp = AEBParams()

    bt = meta["brake_time"]
    bt_txt = f"t={bt:.1f} s" if bt is not None else "未触发"
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(f"{title}（最小间隙 {meta['min_gap']:.1f} m，刹车时刻 {bt_txt}）")
    ax = axes[0]
    ax.plot(t, d, lw=1.5)
    ax.axhline(pp.warn_enter, color="orange", ls="--", lw=1, label=f"warn < {pp.warn_enter:g} m")
    ax.axhline(pp.brake_enter, color="red", ls="--", lw=1, label=f"brake < {pp.brake_enter:g} m")
    ax.set_ylabel("distance (m)")
    ax.legend(loc="upper right")
    ax = axes[1]
    ax.plot(t, v, lw=1.5)
    ax.set_ylabel("speed (m/s)")
    ax = axes[2]
    ax.step(t, brake, where="post", color="red", lw=1.5, label="brake")
    ax.fill_between(t, 0, state, step="post", alpha=0.15, color="blue", label="state")
    ax.set_ylim(-0.1, 2.6)
    ax.set_yticks([0, 1, 2, 3], ["cruise", "warning", "braking", "recover"])
    ax.set_ylabel("brake / state")
    ax.set_xlabel("time (s)")
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(str(out_png), dpi=110)
    plt.close(fig)
    return True


def run_offline(out_dir=ARTIFACTS):
    """离线演示：完整状态链仿真 + 曲线图 + CSV。"""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    history, meta = simulate_aeb_scenario()
    csv_path = out_dir / "aeb_offline_data.csv"
    _write_csv(history, csv_path,
               ["t", "state", "distance_m", "speed_mps", "closing_mps",
                "throttle", "brake", "event"])
    png_path = out_dir / "aeb_offline_curve.png"
    plot_curve(history, meta, png_path)
    print(f"[demo] 离线演示完成（{meta['t_end']:.1f} s）：")
    print(f"  刹车时刻   t = {meta['brake_time']:.2f} s")
    print(f"  最小间隙   {meta['min_gap']:.2f} m")
    print(f"  是否刹停   {meta['stopped']}")
    print(f"  最终状态   {meta['final_state']}")
    print(f"  数据       {csv_path}")
    print(f"  曲线图     {png_path}")
    return 0


def run_live(host="127.0.0.1", port=2000, out_dir=ARTIFACTS):
    """真实 CARLA 演示：生成自车 + 前向障碍车，AEB 触发后截图并导出数据。"""
    from carla_aeb_braking import carla_common as cc
    from carla_aeb_braking.aeb_core import AEBController, frame_min_distance
    import carla

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[demo] 连接 CARLA {host}:{port} ...")
    client, world = cc.connect(host, port)
    ego, tf = cc.spawn_vehicle(world)
    depth = {"f": None}
    cam = cc.make_depth_camera(
        world, ego,
        lambda img: depth.__setitem__("f", (bytes(img.raw_data), img.width, img.height)),
        tick=True)
    print(f"[demo] 自车已生成于 ({tf.location.x:.1f}, {tf.location.y:.1f})")

    # 在自车前方约 35 m 的同一车道生成静止障碍车
    m = world.get_map()
    wp = m.get_waypoint(tf.location)
    if wp is None:
        raise RuntimeError("自车出生点不在车道上，无法定位障碍车")
    obs_wp = wp
    for _ in range(35):
        nxt = obs_wp.next(1.0)
        if not nxt:
            break
        obs_wp = nxt[0]
    obs_bp = world.get_blueprint_library().find("vehicle.audi.tt")
    obstacle = world.try_spawn_actor(obs_bp, obs_wp.transform)
    if obstacle is None:
        raise RuntimeError("无法生成障碍车")
    print(f"[demo] 障碍车已生成于 ({obs_wp.transform.location.x:.1f}, "
          f"{obs_wp.transform.location.y:.1f})")

    ctl = AEBController()
    p = ctl.p
    history = []
    shot_done = False
    stopped_until = None
    try:
        for step in range(int(90.0 / cc.DT)):
            world.tick()
            t = step * cc.DT
            v = cc.get_speed(ego)
            if depth["f"] is not None:
                raw, w, h = depth["f"]
                d = frame_min_distance(raw, w, h, p.valid_min, p.valid_max)
            else:
                d = p.valid_max
            closing = max(v, 0.0)  # 障碍车静止
            state, throttle, brake, event = ctl.tick(d, closing, t=t)
            cc.apply_control(ego, throttle, 0.0, brake)
            history.append((t, state, d, v, closing, throttle, brake, event))
            if event == "brake" and not shot_done:
                print(f"[demo] ★ 紧急制动触发（t={t:.1f}s，d={d:.1f}m，v={v:.1f}m/s）")
                shot_done = _grab_screenshot(world, ego, out_dir / "aeb_live_brake.png")
            # 刹停且障碍仍在前方：持续 2 s 后结束
            if v < 0.2 and d < p.brake_exit:
                if stopped_until is None:
                    stopped_until = t + 2.0
                elif t >= stopped_until:
                    break
            print(f"  t={t:5.1f}s  state={state:8s} d={d:5.1f}m  v={v:4.1f}m/s  "
                  f"brake={brake:.2f}", end="\r")
        print()
    finally:
        for actor in (cam, ego, obstacle):
            try:
                actor.destroy()
            except Exception:  # noqa: BLE001
                pass
        cc.restore_async(world)

    # 导出
    csv_path = out_dir / "aeb_live_data.csv"
    _write_csv(history, csv_path,
               ["t", "state", "distance_m", "speed_mps", "closing_mps",
                "throttle", "brake", "event"])
    meta = {
        "min_gap": min(h[2] for h in history) if history else float("inf"),
        "brake_time": next((h[0] for h in history if h[7] == "brake"), None),
        "stopped": any(h[3] < 0.2 and h[2] < p.brake_exit for h in history[-10:]),
        "final_state": ctl.state,
        "t_end": history[-1][0] if history else 0.0,
    }
    png_path = out_dir / "aeb_live_curve.png"
    plot_curve(history, meta, png_path, title="AEB CARLA 真实场景演示")
    print(f"[demo] CARLA 演示完成（{meta['t_end']:.1f} s）：")
    print(f"  刹车时刻   t = {meta['brake_time']} s")
    print(f"  最小间隙   {meta['min_gap']:.2f} m")
    print(f"  是否刹停   {meta['stopped']}")
    print(f"  最终状态   {meta['final_state']}")
    print(f"  数据       {csv_path}")
    print(f"  曲线图     {png_path}")
    return 0


def _grab_screenshot(world, ego, out_path):
    """在自车后上方挂临时 RGB 相机抓一帧存 PNG（offscreen 下 get_screenshot 易黑屏）。"""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import carla

    shot = {"img": None}

    def cb(img):
        if shot["img"] is None:
            shot["img"] = bytes(img.raw_data)

    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    w_attr = bp.get_attribute("image_size_x")
    h_attr = bp.get_attribute("image_size_y")
    w = int(w_attr.value)
    h = int(h_attr.value)
    bp.set_attribute("image_size_x", "800")
    bp.set_attribute("image_size_y", "450")
    tf = ego.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    cam_tf = carla.Transform(
        carla.Location(x=tf.location.x - 6.0 * math.cos(yaw),
                       y=tf.location.y - 6.0 * math.sin(yaw),
                       z=tf.location.z + 2.5),
        carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw))
    cam = world.spawn_actor(bp, cam_tf)
    try:
        cam.listen(cb)
        world.tick()
    finally:
        try:
            cam.stop()
            cam.destroy()
        except Exception:  # noqa: BLE001
            pass
    if shot["img"] is None:
        print("[demo] 截图失败（未收到图像）")
        return False
    arr = np.frombuffer(shot["img"], dtype=np.uint8).reshape(450, 800, 4)
    rgb = arr[:, :, :3][:, :, ::-1].copy()
    plt.imsave(str(out_path), rgb)
    print(f"[demo] 刹车瞬间截图已保存 {out_path}")
    return True


if __name__ == "__main__":
    import sys
    if "--live" in sys.argv:
        raise SystemExit(run_live())
    raise SystemExit(run_offline())
