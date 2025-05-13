import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('decentralized_auction_mm')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default=os.path.join(pkg_dir, 'worlds', 'assembly_environment.world'))
    
    # Configuration files
    auction_params_file = os.path.join(pkg_dir, 'config', 'auction_params.yaml')
    robot_config_file = os.path.join(pkg_dir, 'config', 'robot_config.yaml')
    task_config_file = os.path.join(pkg_dir, 'config', 'task_config.yaml')
    
    # Launch Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
        }.items(),
    )
    
    # Environment variables
    env_vars = [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', 
                              os.path.join(pkg_dir, 'models')),
    ]
    
    # Launch two TurtleBot3 robots with OpenMANIPULATOR-X
    robot1_launch = IncludeLaunchDescription(
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
    
    robot2_launch = IncludeLaunchDescription(
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
    
    # Task generator node
    task_generator_node = Node(
        package='decentralized_auction_mm',
        executable='task_generator_node',
        name='task_generator',
        parameters=[
            {'use_sim_time': use_sim_time},
            task_config_file
        ],
        output='screen',
    )
    
    # Auction system node
    auction_node = Node(
        package='decentralized_auction_mm',
        executable='auction_node',
        name='auction_system',
        parameters=[
            {'use_sim_time': use_sim_time},
            auction_params_file
        ],
        output='screen',
    )
    
    # Visualization node
    visualization_node = Node(
        package='decentralized_auction_mm',
        executable='visualize_results.py',
        name='visualization',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
    )
    
    # Create the launch description
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'world_file',
            default_value=os.path.join(pkg_dir, 'worlds', 'assembly_environment.world'),
            description='Gazebo world file'),
            
        # Environment variables
        *env_vars,
        
        # Launch Gazebo and robots
        gazebo_launch,
        robot1_launch,
        robot2_launch,
        
        # Launch task generator and auction system
        task_generator_node,
        auction_node,
        visualization_node
    ])