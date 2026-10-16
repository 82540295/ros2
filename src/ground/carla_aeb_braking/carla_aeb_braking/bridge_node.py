#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA ↔ ROS 2 桥接节点：生成自车 + 前向深度相机。

职责：
  * 连接 CARLA、生成自车与前向深度相机（同步模式 20 Hz 步进）
  * 每 tick：深度帧 → /aeb/obstacle_distance；车速/位姿 → /aeb/ego_speed、/aeb/odometry
  * 订阅 /aeb/control（[throttle, steer, brake]）并施加到车辆
  * 退出时销毁传感器/自车，并把世界恢复为异步模式

话题：
  发布  /aeb/obstacle_distance  (std_msgs/Float32, 米；无障碍时为 valid_max)
        /aeb/ego_speed          (std_msgs/Float32, m/s)
        /aeb/odometry           (nav_msgs/Odometry, map 系)
  订阅  /aeb/control            (std_msgs/Float32MultiArray, [throttle, steer, brake])

启动：  ros2 launch carla_aeb_braking main.launch.py host:=<CARLA 所在机器 IP>
"""
import math
import os
import sys

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

# 允许以脚本方式直接运行（launch 传入时包尚未安装）
_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from carla_aeb_braking import carla_common as cc            # noqa: E402
from carla_aeb_braking.aeb_core import frame_min_distance   # noqa: E402


class AEBBridgeNode(Node):
    """CARLA 桥接节点：传感器 → 话题，控制话题 → 车辆。"""

    def __init__(self):
        super().__init__("aeb_bridge_node")

        # ---------------- 参数 ----------------
        self.declare_parameter("host", cc.DEFAULT_HOST)
        self.declare_parameter("port", cc.DEFAULT_PORT)
        self.declare_parameter("town", cc.DEFAULT_MAP)
        self.declare_parameter("ego_blueprint", cc.EGO_BLUEPRINT)
        self.declare_parameter("depth_width", 640)
        self.declare_parameter("depth_height", 360)
        self.declare_parameter("depth_fov", 90)
        self.declare_parameter("valid_min", 0.3)
        self.declare_parameter("valid_max", 80.0)
        self.declare_parameter("fixed_update", cc.DT)

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        town = self.get_parameter("town").value
        blueprint = self.get_parameter("ego_blueprint").value
        self.depth_width = int(self.get_parameter("depth_width").value)
        self.depth_height = int(self.get_parameter("depth_height").value)
        self.depth_fov = int(self.get_parameter("depth_fov").value)
        self.valid_min = float(self.get_parameter("valid_min").value)
        self.valid_max = float(self.get_parameter("valid_max").value)
        dt = float(self.get_parameter("fixed_update").value)

        # ---------------- 话题 ----------------
        self.pub_distance = self.create_publisher(Float32, "/aeb/obstacle_distance", 10)
        self.pub_speed = self.create_publisher(Float32, "/aeb/ego_speed", 10)
        self.pub_odom = self.create_publisher(Odometry, "/aeb/odometry", 10)
        self.create_subscription(Float32MultiArray, "/aeb/control", self._on_control, 10)

        # ---------------- CARLA ----------------
        self._depth = None          # (raw_bytes, width, height)
        self._control = (0.0, 0.0, 0.0)
        self.client = None
        self.world = None
        self.vehicle = None
        self.sensor = None
        self._connect(host, port, town, blueprint)
        self.create_timer(dt, self._on_timer)

    # ------------------------------------------------------------------ 连接
    def _connect(self, host, port, town, blueprint):
        try:
            self.client, self.world = cc.connect(host, port, town)
            self.vehicle, tf = cc.spawn_vehicle(self.world, blueprint)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"连接 CARLA 失败：{exc}")
            self.get_logger().error(
                "请先启动 CARLA 服务端（端口 2000）；跨机器时 host:=<CARLA 所在机器 IP>")
            return
        self.sensor = cc.make_depth_camera(
            self.world, self.vehicle,
            lambda img: self._on_depth(img),
            width=self.depth_width, height=self.depth_height,
            fov=self.depth_fov, tick=True)
        self.get_logger().info(
            f"自车已生成于 ({tf.location.x:.1f}, {tf.location.y:.1f}, {tf.location.z:.1f})，"
            f"深度相机 {self.depth_width}x{self.depth_height} 就绪")

    # ------------------------------------------------------------------ 回调
    def _on_depth(self, img):
        # bytes() 复制缓冲：CARLA 会复用 raw_data 底层内存
        self._depth = (bytes(img.raw_data), img.width, img.height)

    def _on_control(self, msg):
        d = list(msg.data)
        if len(d) >= 3:
            self._control = (float(d[0]), float(d[1]), float(d[2]))

    # ------------------------------------------------------------------ 主循环
    def _on_timer(self):
        if self.world is None or self.vehicle is None:
            return
        if self._depth is not None:
            raw, w, h = self._depth
            dist = frame_min_distance(raw, w, h,
                                      valid_min=self.valid_min,
                                      valid_max=self.valid_max)
            self.pub_distance.publish(Float32(data=float(dist)))
        speed = cc.get_speed(self.vehicle)
        self.pub_speed.publish(Float32(data=float(speed)))
        self._publish_odom(speed)
        cc.apply_control(self.vehicle, *self._control)
        self.world.tick()

    def _publish_odom(self, speed):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "ego_vehicle"
        x, y, z = cc.get_location(self.vehicle)
        yaw = cc.get_yaw(self.vehicle)
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        msg.twist.twist.linear.x = float(speed)
        self.pub_odom.publish(msg)

    # ------------------------------------------------------------------ 清理
    def destroy_node(self):
        for actor in (self.sensor, self.vehicle):
            if actor is not None:
                try:
                    actor.stop()
                    actor.destroy()
                except Exception:  # noqa: BLE001
                    pass
        if self.world is not None and cc.restore_async(self.world):
            self.get_logger().info("已销毁传感器与自车，世界已恢复异步模式。")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AEBBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
