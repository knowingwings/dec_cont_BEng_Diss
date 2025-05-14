import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('decentralized_auction_mm')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Configuration files
    auction_params_file = os.path.join(pkg_dir, 'config', 'auction_params.yaml')
    task_config_file = os.path.join(pkg_dir, 'config', 'task_config.yaml')
    
    # Launch actions
    launch_actions = []
    
    # Launch arguments
    launch_actions.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true')
    )
    
    # Launch Gazebo simulation
    launch_actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={
                'world': os.path.join(pkg_dir, 'worlds', 'assembly_environment.world'),
                'verbose': 'true',
            }.items(),
        )
    )
    
    # Launch TurtleBot3 robots with OpenMANIPULATOR-X if available
    launch_actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_dir, 'launch', 'single_robot.launch.py')
            ]),
            launch_arguments={
                'robot_id': '1',
                'x_pos': '0.5',
                'y_pos': '0.5',
                'z_pos': '0.0',
                'robot_name': 'robot1',
                'use_sim_time': use_sim_time,
            }.items(),
        )
    )
    
    launch_actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_dir, 'launch', 'single_robot.launch.py')
            ]),
            launch_arguments={
                'robot_id': '2',
                'x_pos': '3.5',
                'y_pos': '3.5',
                'z_pos': '0.0',
                'robot_name': 'robot2',
                'use_sim_time': use_sim_time,
            }.items(),
        )
    )
    
    # Try to launch MoveIt integration if available
    try:
        moveit_launch_file = os.path.join(
            get_package_share_directory('open_manipulator_x_moveit_config'),
            'launch', 'moveit_core.launch.py')
        
        # Check if file exists
        if os.path.exists(moveit_launch_file):
            launch_actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([moveit_launch_file]),
                )
            )
    except:
        # MoveIt package not available, continue without
        pass
    
    # Task generator node
    launch_actions.append(
        Node(
            package='decentralized_auction_mm',
            executable='task_generator_node',
            name='task_generator',
            parameters=[
                {'use_sim_time': use_sim_time},
                task_config_file
            ],
            output='screen',
        )
    )
    
    # Auction system node
    launch_actions.append(
        Node(
            package='decentralized_auction_mm',
            executable='auction_node',
            name='auction_system',
            parameters=[
                {'use_sim_time': use_sim_time},
                auction_params_file
            ],
            output='screen',
        )
    )
    
    # Visualization node
    launch_actions.append(
        Node(
            package='decentralized_auction_mm',
            executable='visualize_results.py',
            name='visualization',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
        )
    )
    
    return LaunchDescription(launch_actions)