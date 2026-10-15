from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
        # The mission controller and teleop nodes are added in later commits.
        Node(package='uav_waypoint_mission', executable='uav_state_source', output='screen',
             arguments=['--host', LaunchConfiguration('host'), '--vehicle', LaunchConfiguration('vehicle')]),
    ])
