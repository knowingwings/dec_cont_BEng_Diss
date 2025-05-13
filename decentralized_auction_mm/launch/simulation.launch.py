import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('decentralized_auction_mm')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': os.path.join(pkg_dir, 'worlds', 'assembly_environment.world'),
        }.items(),
    )
    
    # Launch dual robots
    robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'dual_robots.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )
    
    # Launch auction system
    auction_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'auction_system.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )
    
    return LaunchDescription([
        gazebo_launch,
        robots_launch,
        auction_launch
    ])