"""ROS2 nodes bridging the waypoint mission core with the AirSim/CarlaAir simulator.

`StateSource` connects to the simulator RPC server (port 41451), performs a
startup ping self-check, enables API control and publishes the vehicle state at
10 Hz:
  /uav/odometry  nav_msgs/Odometry  ENU map frame, FLU body frame
  /uav/state     std_msgs/String    versioned JSON (NED/FRD raw values)
"""
import argparse
import json
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .coords import body_velocity_to_world_ned, enu_flu_orientation, ned_to_enu, ned_angular_to_flu, ned_velocity_to_flu
from .mission import Mission, MissionController, State
from .planner import manual_active, plan_airsim_command, twist_is_release

DT = 0.1  # 10 Hz
AUTOSTART_DELAY_S = 3.0
TELEOP_SPEED = 1.5  # m/s per axis while a key is held


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
        self.latest_quat = (1.0, 0.0, 0.0, 0.0)
        self.latest_cmd = None
        self.latest_cmd_t = 0.0
        self.disarmed = False
        self.prev_state = State.IDLE
        self.sub_odom = self.create_subscription(Odometry, '/uav/odometry', self._on_odom, 10)
        self.sub_cmd = self.create_subscription(Twist, '/uav/cmd_vel', self._on_cmd_vel, 10)
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
        o = msg.pose.pose.orientation
        self.latest = ((p.x, p.y, p.z), (v.x, v.y, v.z))
        self.latest_quat = (o.w, o.x, o.y, o.z)

    def _on_cmd_vel(self, msg):
        t = msg.twist
        if twist_is_release(t.linear.x, t.linear.y, t.linear.z):
            self.latest_cmd = None  # explicit release (teleop R key)
        else:
            self.latest_cmd = (t.linear.x, t.linear.y, t.linear.z)
            self.latest_cmd_t = time.monotonic()

    def _control(self):
        if self.latest is None:
            return
        if manual_active(time.monotonic(), self.latest_cmd_t):
            v_ned = body_velocity_to_world_ned(self.latest_cmd, self.latest_quat)
            self.client.moveByVelocityAsync(v_ned[0], v_ned[1], v_ned[2],
                                            vehicle_name=self.vehicle,
                                            drivetrain=self.airsim.MultirotorDrivetrain)
            status = dict(schema_version=1, state='manual', manual=True,
                          waypoint_index=self.ctl.waypoint_index,
                          waypoint_count=len(self.mission.waypoints), target_enu=None)
            self.pub_status.publish(String(data=json.dumps(status)))
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
        status = dict(schema_version=1, state=self.ctl.state.value, manual=False,
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


class TeleopNode(Node):
    """Keyboard teleop: publishes body-frame (FLU) velocities on /uav/cmd_vel.

    Keys (motion continues while held):
        W/S forward/backward   A/D left/right   Q/E ascend/descend
        F  abort the mission (vertical landing)
        R  release manual control immediately (the mission resumes)
        Esc exit
    While a motion key is held the mission controller runs in manual mode
    (state machine frozen); after release it resumes automatically.
    """

    def __init__(self):
        super().__init__('uav_teleop')
        self.pub = self.create_publisher(Twist, '/uav/cmd_vel', 10)
        self.srv_abort = Client(self, Trigger, '/uav/mission/abort')
        self.keys = {}
        self._stop = False
        self._fd = None
        self._old = None
        self.timer = self.create_timer(DT, self._publish)
        if self._enter_raw_mode():
            threading.Thread(target=self._read_keys, daemon=True).start()
            self.get_logger().info('keyboard teleop ready: WASD/QE move, F abort, R release, Esc exit')
        else:
            self.get_logger().warn('no tty on stdin - key reading disabled')

    def _enter_raw_mode(self):
        try:
            import termios
            import tty
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            return True
        except Exception:
            self._fd = None
            return False

    def _leave_raw_mode(self):
        if self._fd is not None:
            try:
                import termios
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
            except Exception:
                pass

    def _read_keys(self):
        while not self._stop:
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                self.get_logger().info('Esc - exiting teleop')
                self._stop = True
                rclpy.shutdown()
                return
            if ch:
                self.keys[ch.lower()] = True

    def _publish(self):
        if self.keys.pop('f', False) and self.srv_abort.service_is_ready():
            self.srv_abort.call_async(Trigger.Request())
            self.get_logger().info('abort requested - landing from current position')
        if self.keys.pop('r', False):
            self.keys = {}
            self.pub.publish(Twist())  # zero twist = explicit release
            self.get_logger().info('manual released - mission resumes')
        if not any(self.keys.get(k) for k in 'wasdqe'):
            return
        t = Twist()
        t.linear.x = self._axis('w', 's')
        t.linear.y = self._axis('a', 'd')
        t.linear.z = self._axis('e', 'q')
        self.pub.publish(t)

    def _axis(self, pos, neg):
        if self.keys.get(pos):
            return TELEOP_SPEED
        if self.keys.get(neg):
            return -TELEOP_SPEED
        return 0.0


def teleop_main():
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop = True
        node._leave_raw_mode()
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
