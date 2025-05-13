import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('decentralized_auction_mm')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Launch robot 1
    robot1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'single_robot.launch.py')
        ]),
        launch_arguments={
            'robot_id': '1',
            'robot_name': 'robot1',
            'x_pos': '0.5',
            'y_pos': '0.5',
            'z_pos': '0.0',
            'use_sim_time': use_sim_time,
        }.items(),
    )
    
    # Launch robot 2
    robot2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'single_robot.launch.py')
        ]),
        launch_arguments={
            'robot_id': '2',
            'robot_name': 'robot2',
            'x_pos': '3.5',
            'y_pos': '3.5',
            'z_pos': '0.0',
            'use_sim_time': use_sim_time,
        }.items(),
    )
    
    return LaunchDescription([
        robot1_launch,
        robot2_launch
    ])