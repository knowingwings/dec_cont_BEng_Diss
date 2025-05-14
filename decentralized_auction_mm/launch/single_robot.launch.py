import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('decentralized_auction_mm')
    
    # Launch arguments
    robot_id = LaunchConfiguration('robot_id')
    robot_name = LaunchConfiguration('robot_name')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Configuration files
    robot_config_file = os.path.join(pkg_dir, 'config', 'robot_config.yaml')
    auction_params_file = os.path.join(pkg_dir, 'config', 'auction_params.yaml')
    
    # Create action to launch the robot nodes within a namespace
    launch_robot_nodes = GroupAction([
        # Push robot namespace
        PushRosNamespace(robot_name),
        
        # Spawn TurtleBot3 with OpenMANIPULATOR-X
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=[
                '-entity', robot_name,
                '-file', os.path.join(get_package_share_directory('turtlebot3_description'), 
                                     'urdf', 'turtlebot3_waffle_pi.urdf'),
                '-x', x_pos,
                '-y', y_pos,
                '-z', z_pos,
                '-R', '0.0',
                '-P', '0.0',
                '-Y', '0.0'
            ],
            output='screen'
        ),
        
        # Spawn OpenMANIPULATOR-X on TurtleBot3
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_manipulator',
            arguments=[
                '-entity', [robot_name, '_manipulator'],
                '-file', os.path.join(get_package_share_directory('open_manipulator_x_description'), 
                                     'urdf', 'open_manipulator.urdf'),
                '-x', x_pos,
                '-y', y_pos,
                '-z', TextSubstitution(text=str(float(z_pos.perform(None)) + 0.15)),  # Add offset for mounting on TurtleBot
                '-R', '0.0',
                '-P', '0.0',
                '-Y', '0.0',
                '-reference_frame', [robot_name, '::base_link'],
            ],
            output='screen'
        ),
        
        # Robot controller nodes
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # TurtleBot controllers
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_drive',
            name='turtlebot3_drive',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # OpenMANIPULATOR-X controllers
        Node(
            package='open_manipulator_x_controller',
            executable='open_manipulator_x_controller',
            name='open_manipulator_x_controller',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # Auction robot node
        Node(
            package='decentralized_auction_mm',
            executable='robot_node',
            name='robot_node',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_id': robot_id},
                robot_config_file,
                auction_params_file
            ],
            output='screen'
        ),
    ])
    
    # Create the launch description
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'robot_id',
            default_value='1',
            description='Robot ID'),
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot1',
            description='Robot name'),
        DeclareLaunchArgument(
            'x_pos',
            default_value='0.0',
            description='X position'),
        DeclareLaunchArgument(
            'y_pos',
            default_value='0.0',
            description='Y position'),
        DeclareLaunchArgument(
            'z_pos',
            default_value='0.0',
            description='Z position'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        # Launch robot nodes
        launch_robot_nodes
    ])