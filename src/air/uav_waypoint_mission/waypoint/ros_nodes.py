"""ROS2 nodes bridging the waypoint mission core with the AirSim/CarlaAir simulator.

`StateSource` connects to the simulator RPC server (port 41451), performs a
startup ping self-check, enables API control and publishes the vehicle state at
10 Hz:
  /uav/odometry  nav_msgs/Odometry  ENU map frame, FLU body frame
  /uav/state     std_msgs/String    versioned JSON (NED/FRD raw values)
"""
import argparse
import json

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from .coords import enu_flu_orientation, ned_to_enu, ned_angular_to_flu, ned_velocity_to_flu

DT = 0.1  # 10 Hz


def _stamp(ns):
    return Time(sec=int(ns // 1000000000), nanosec=int(ns % 1000000000))


class StateSource(Node):
    def __init__(self, args):
        super().__init__('uav_state_source')
        import airsim  # imported here so the package stays importable offline
        self.vehicle = args.vehicle
        self.client = airsim.MultirotorClient(ip=args.host, timeout_value=2)
        if not self.client.ping():
            raise RuntimeError('AirSim ping failed at %s:41451 - is the simulator running?' % args.host)
        self.client.enableApiControl(True, self.vehicle)
        self.pub_odom = self.create_publisher(Odometry, '/uav/odometry', 10)
        self.pub_state = self.create_publisher(String, '/uav/state', 10)
        self.timer = self.create_timer(DT, self._live)
        self.get_logger().info('state source connected to %s (vehicle %s)' % (args.host, self.vehicle))

    def _live(self):
        try:
            s = self.client.getMultirotorState(vehicle_name=self.vehicle)
        except Exception as e:
            self.get_logger().error('AirSim state read failed: %s' % e)
            return
        ns = int(s.timestamp)
        k = s.kinematics_estimated
        q = [k.pose.orientation.w, k.pose.orientation.x, k.pose.orientation.y, k.pose.orientation.z]
        pos = ned_to_enu([k.pose.position.x, k.pose.position.y, k.pose.position.z])
        v_flu = ned_velocity_to_flu([k.velocity.x, k.velocity.y, k.velocity.z], q)
        w_flu = ned_angular_to_flu([k.angular_velocity.x, k.angular_velocity.y, k.angular_velocity.z])

        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = _stamp(ns)
        odom.child_frame_id = 'drone_flu'
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = pos
        oq = enu_flu_orientation(q)
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, \
            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z = (float(v) for v in oq)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = (float(v) for v in v_flu)
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = (float(v) for v in w_flu)
        self.pub_odom.publish(odom)

        state = dict(schema_version=1, timestamp_ns=ns, frame='NED_FRD', vehicle=self.vehicle,
                     position_ned=[k.pose.position.x, k.pose.position.y, k.pose.position.z],
                     velocity_ned=[k.velocity.x, k.velocity.y, k.velocity.z])
        self.pub_state.publish(String(data=json.dumps(state)))


def source_main():
    parser = argparse.ArgumentParser(description='Publish the AirSim vehicle state as ROS2 topics')
    parser.add_argument('--host', default='192.168.122.1', help='IP of the machine running the simulator')
    parser.add_argument('--vehicle', default='WaypointDrone', help='AirSim vehicle name')
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = StateSource(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
