"""ROS2 nodes bridging the waypoint mission core with the AirSim/CarlaAir simulator.

`StateSource` connects to the simulator RPC server (port 41451), performs a
startup ping self-check, enables API control and publishes the vehicle state at
10 Hz:
  /uav/odometry  nav_msgs/Odometry  ENU map frame, FLU body frame
  /uav/state     std_msgs/String    versioned JSON (NED/FRD raw values)
"""
import argparse
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .coords import enu_flu_orientation, ned_to_enu, ned_angular_to_flu, ned_velocity_to_flu
from .mission import Mission, MissionController, State
from .planner import plan_airsim_command

DT = 0.1  # 10 Hz
AUTOSTART_DELAY_S = 3.0


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


class MissionControllerNode(Node):
    """Drives the waypoint mission state machine against the simulator.

    Subscribes to /uav/odometry (StateSource), feeds the pure MissionController
    and sends moveOnPositionAsync commands to AirSim. Services (std_srvs/Trigger):
    /uav/mission/arm, /uav/mission/go, /uav/mission/abort. Publishes
    /uav/mission_status (versioned JSON) at the control rate.
    """

    def __init__(self, args):
        super().__init__('uav_mission_controller')
        import airsim  # imported here so the package stays importable offline
        self.airsim = airsim
        self.vehicle = args.vehicle
        self.client = airsim.MultirotorClient(ip=args.host, timeout_value=2)
        if not self.client.ping():
            raise RuntimeError('AirSim ping failed at %s:41451 - is the simulator running?' % args.host)
        self.client.enableApiControl(True, self.vehicle)

        self.mission = Mission.from_file(args.mission)
        self.ctl = MissionController(self.mission)
        self.latest = None
        self.disarmed = False
        self.prev_state = State.IDLE
        self.sub_odom = self.create_subscription(Odometry, '/uav/odometry', self._on_odom, 10)
        self.pub_status = self.create_publisher(String, '/uav/mission_status', 10)
        self.srv_arm = self.create_service(Trigger, '/uav/mission/arm', self._srv_arm)
        self.srv_go = self.create_service(Trigger, '/uav/mission/go', self._srv_go)
        self.srv_abort = self.create_service(Trigger, '/uav/mission/abort', self._srv_abort)
        self.timer = self.create_timer(DT, self._control)
        if args.autostart:
            self.get_logger().info('autostart: mission starts in %.0f s' % AUTOSTART_DELAY_S)
            self._autostart_timer = self.create_timer(AUTOSTART_DELAY_S, self._autostart)
        else:
            self.get_logger().info('mission loaded (%s, %d waypoints); call /uav/mission/arm then /uav/mission/go',
                                   self.mission.name, len(self.mission.waypoints))

    def _autostart(self):
        self._autostart_timer.cancel()
        self.ctl.arm()
        self.ctl.go()
        self.get_logger().info('mission started: %s' % self.mission.name)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.latest = ((p.x, p.y, p.z), (v.x, v.y, v.z))

    def _control(self):
        if self.latest is None:
            return
        pos, vel = self.latest
        cmd = self.ctl.tick(pos, vel, DT)
        action, target_ned, speed = plan_airsim_command(cmd)
        if action == 'move' and target_ned is not None:
            self.client.moveOnPositionAsync(target_ned[0], target_ned[1], target_ned[2],
                                            vehicle_name=self.vehicle, velocity=speed,
                                            drivetrain=self.airsim.MultirotorDrivetrain)
        if self.ctl.state is State.FAILED and self.prev_state is not State.FAILED:
            self.get_logger().error('mission FAILED at waypoint %d - holding position' % self.ctl.waypoint_index)
        elif self.ctl.state is State.LANDED and not self.disarmed:
            self.client.armControlMode(False, vehicle_name=self.vehicle)
            self.disarmed = True
            self.get_logger().info('mission complete: %s - landed and disarmed' % self.mission.name)
        self.prev_state = self.ctl.state
        status = dict(schema_version=1, state=self.ctl.state.value,
                      waypoint_index=self.ctl.waypoint_index,
                      waypoint_count=len(self.mission.waypoints),
                      target_enu=list(cmd.target) if cmd.target is not None else None)
        self.pub_status.publish(String(data=json.dumps(status)))

    def _srv_arm(self, request, response):
        self.ctl.arm()
        response.success = self.ctl.state is State.ARMED
        response.message = 'state=' + self.ctl.state.value
        return response

    def _srv_go(self, request, response):
        self.ctl.go()
        response.success = self.ctl.state is State.TAKING_OFF
        response.message = 'state=' + self.ctl.state.value
        return response

    def _srv_abort(self, request, response):
        self.ctl.abort()
        response.success = self.ctl.state is State.LANDING
        response.message = 'state=' + self.ctl.state.value
        return response


def mission_main():
    parser = argparse.ArgumentParser(description='Drive the waypoint mission against the simulator')
    parser.add_argument('--host', default='192.168.122.1', help='IP of the machine running the simulator')
    parser.add_argument('--vehicle', default='WaypointDrone', help='AirSim vehicle name')
    parser.add_argument('--mission', default='', help='Absolute path of the mission YAML (default: package config)')
    parser.add_argument('--autostart', default='false', help='Arm and start the mission automatically (true/false)')
    a, _ = parser.parse_known_args()
    a.mission = a.mission or str(Path(__file__).resolve().parents[1] / 'config' / 'mission.yaml')
    a.autostart = a.autostart.lower() in ('1', 'true', 'yes')
    rclpy.init()
    node = MissionControllerNode(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


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
