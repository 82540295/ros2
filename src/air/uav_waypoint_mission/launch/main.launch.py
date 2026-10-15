from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = str(Path(get_package_share_directory('uav_waypoint_mission')) / 'config' / 'mission.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='192.168.122.1',
            description='IP address of the machine running the AirSim/CarlaAir simulator (RPC port 41451)'),
        DeclareLaunchArgument('vehicle', default_value='WaypointDrone',
            description='AirSim vehicle name; must match config/airsim_settings.json'),
        DeclareLaunchArgument('mission', default_value=share,
            description='Absolute path of the waypoint mission YAML file'),
        DeclareLaunchArgument('rviz', default_value='false',
            description='Start RViz2 with the mission view (true/false)'),
        # The simulator-facing nodes (state source / mission controller / teleop)
        # are added in later commits of this module.
        LogInfo(msg=['[uav_waypoint_mission] skeleton ready: host=', LaunchConfiguration('host'),
                     ' vehicle=', LaunchConfiguration('vehicle')]),
    ])
