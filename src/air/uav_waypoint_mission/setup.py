from glob import glob
from setuptools import setup

setup(name='uav_waypoint_mission', version='0.1.0', packages=['waypoint'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/uav_waypoint_mission']),
                ('share/uav_waypoint_mission', ['package.xml']),
                ('share/uav_waypoint_mission/launch', glob('launch/*.py')),
                ('share/uav_waypoint_mission/config', glob('config/*'))],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    description='Drone waypoint mission flight on AirSim/CarlaAir with ROS 2 Humble',
    maintainer='825404295', maintainer_email='825404295@qq.com', license='Apache-2.0',
    entry_points={'console_scripts': ['uav_state_source=waypoint.ros_nodes:source_main']})
