# decentralised_controller.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    """Generate launch description for the decentralized control system with experimental parameters"""
    
    # Launch arguments
    robot1_id = LaunchConfiguration('robot1_id', default='1')
    robot2_id = LaunchConfiguration('robot2_id', default='2')
    num_tasks = LaunchConfiguration('num_tasks', default='10')
    dependency_prob = LaunchConfiguration('dependency_prob', default='0.3')
    
    # Add experimental parameters
    delay_ms = LaunchConfiguration('delay_ms', default='0')
    packet_loss = LaunchConfiguration('packet_loss', default='0.0')
    epsilon = LaunchConfiguration('epsilon', default='0.05')
    experiment_id = LaunchConfiguration('experiment_id', default='default')
    
    # Add comparative analysis parameter
    run_comparative = LaunchConfiguration('run_comparative', default='true')
    comparative_method = LaunchConfiguration('comparative_method', default='CENTRALIZED')
    
    # Add Gazebo model arguments
    robot1_model = LaunchConfiguration('robot1_model', default='waffle_pi')
    robot2_model = LaunchConfiguration('robot2_model', default='waffle_pi')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'robot1_id',
            default_value='1',
            description='ID for robot 1'),
        
        DeclareLaunchArgument(
            'robot2_id',
            default_value='2',
            description='ID for robot 2'),
        
        DeclareLaunchArgument(
            'num_tasks',
            default_value='10',
            description='Number of tasks to generate'),
        
        DeclareLaunchArgument(
            'dependency_prob',
            default_value='0.3',
            description='Probability of adding a task dependency'),
        
        DeclareLaunchArgument(
            'robot1_model',
            default_value='waffle_pi',
            description='TurtleBot3 model for robot 1'),
        
        DeclareLaunchArgument(
            'robot2_model',
            default_value='waffle_pi',
            description='TurtleBot3 model for robot 2'),
            
        # Declare experimental parameters
        DeclareLaunchArgument(
            'delay_ms',
            default_value='0',
            description='Communication delay in milliseconds'),
            
        DeclareLaunchArgument(
            'packet_loss',
            default_value='0.0',
            description='Probability of packet loss'),
            
        DeclareLaunchArgument(
            'epsilon',
            default_value='0.05',
            description='Minimum bid increment for auction algorithm'),
            
        DeclareLaunchArgument(
            'experiment_id',
            default_value='default',
            description='Unique identifier for this experiment run'),
            
        # Comparative analysis parameters
        DeclareLaunchArgument(
            'run_comparative',
            default_value='true',
            description='Whether to run comparative analysis'),
            
        DeclareLaunchArgument(
            'comparative_method',
            default_value='CENTRALIZED',
            description='Method to use for comparative analysis'),
        
        # Launch Gazebo simulation with two TurtleBot3 robots
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', 
                 '-s', 'libgazebo_ros_factory.so', 
                 '/opt/ros/humble/share/gazebo_ros/worlds/empty.world'],
            output='screen'),
        
        # Spawn Robot 1
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot1', 
                '-file', f'/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_{robot1_model}/model.sdf',
                '-x', '1.0',
                '-y', '1.0',
                '-z', '0.0'
            ],
            output='screen'),
        
        # Spawn Robot 2
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot2', 
                '-file', f'/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_{robot2_model}/model.sdf',
                '-x', '3.0',
                '-y', '3.0',
                '-z', '0.0'
            ],
            output='screen'),
        
        # Load OpenManipulator for Robot 1
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'manipulator1', 
                '-file', '/opt/ros/humble/share/open_manipulator_x_description/urdf/open_manipulator_x.urdf.xacro',
                '-x', '1.0',
                '-y', '1.0',
                '-z', '0.3'
            ],
            output='screen'),
            
        # Load OpenManipulator for Robot 2
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'manipulator2', 
                '-file', '/opt/ros/humble/share/open_manipulator_x_description/urdf/open_manipulator_x.urdf.xacro',
                '-x', '3.0',
                '-y', '3.0',
                '-z', '0.3'
            ],
            output='screen'),
            
        # Communication Middleware
        Node(
            package='decentralized_control',
            executable='communication_middleware',
            name='communication_middleware',
            parameters=[{
                'delay_mean_ms': delay_ms,
                'delay_stddev_ms': LaunchConfiguration('delay_stddev_ms', default='25'),
                'packet_loss_prob': packet_loss
            }],
            output='screen'),
            
        # Task Manager Node
        Node(
            package='decentralized_control',
            executable='task_manager_node',
            name='task_manager_node',
            parameters=[{
                'num_tasks': num_tasks,
                'environment_size': [4.0, 4.0, 2.0],
                'dependency_probability': dependency_prob
            }],
            output='screen'),
            
        # Robot 1 nodes
        Node(
            package='decentralized_control',
            executable='auction_node',
            name='auction_node_robot1',
            parameters=[{
                'robot_id': robot1_id,
                'epsilon': epsilon,
                'alpha': [0.8, 0.3, 1.0, 1.2, 0.2],
                'gamma': 0.5,
                'lambda': 0.1,
                'beta': [2.0, 1.5]
            }],
            output='screen'),
            
        Node(
            package='decentralized_control',
            executable='consensus_node',
            name='consensus_node_robot1',
            parameters=[{
                'robot_id': robot1_id,
                'gamma': 0.5,
                'lambda': 0.1
            }],
            output='screen'),
            
        Node(
            package='decentralized_control',
            executable='recovery_node',
            name='recovery_node_robot1',
            parameters=[{
                'robot_id': robot1_id,
                'heartbeat_timeout': 3.0,
                'progress_monitoring_interval': 5.0,
                'progress_threshold': 0.05
            }],
            output='screen'),
            
        Node(
            package='decentralized_control',
            executable='execution_controller',
            name='execution_controller_robot1',
            parameters=[{
                'robot_id': robot1_id
            }],
            output='screen'),
            
        # Robot 2 nodes
        Node(
            package='decentralized_control',
            executable='auction_node',
            name='auction_node_robot2',
            parameters=[{
                'robot_id': robot2_id,
                'epsilon': epsilon,
                'alpha': [0.8, 0.3, 1.0, 1.2, 0.2],
                'gamma': 0.5,
                'lambda': 0.1,
                'beta': [2.0, 1.5]
            }],
            output='screen'),
            
        Node(
            package='decentralized_control',
            executable='consensus_node',
            name='consensus_node_robot2',
            parameters=[{
                'robot_id': robot2_id,
                'gamma': 0.5,
                'lambda': 0.1
            }],
            output='screen'),
            
        Node(
            package='decentralized_control',
            executable='recovery_node',
            name='recovery_node_robot2',
            parameters=[{
                'robot_id': robot2_id,
                'heartbeat_timeout': 3.0,
                'progress_monitoring_interval': 5.0,
                'progress_threshold': 0.05
            }],
            output='screen'),
            
        Node(
            package='decentralized_control',
            executable='execution_controller',
            name='execution_controller_robot2',
            parameters=[{
                'robot_id': robot2_id
            }],
            output='screen'),
            
        # Metrics Collector
        Node(
            package='decentralized_control',
            executable='metrics_collector',
            name='metrics_collector',
            parameters=[{
                'output_dir': LaunchConfiguration('output_dir', default='/tmp/decentralized_control/results'),
                'experiment_id': experiment_id,
                'num_tasks': num_tasks,
                'delay_ms': delay_ms,
                'packet_loss': packet_loss,
                'epsilon': epsilon
            }],
            output='screen'),
            
        # Comparative Analysis
        Node(
            package='decentralized_control',
            executable='comparative_analysis',
            name='comparative_analysis',
            condition=IfCondition(run_comparative),
            parameters=[{
                'output_dir': LaunchConfiguration('output_dir', default='/tmp/decentralized_control/results'),
                'allocation_method': comparative_method,
                'num_tasks': num_tasks,
                'num_robots': 2
            }],
            output='screen'),
            
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/opt/ros/humble/share/turtlebot3_gazebo/rviz/turtlebot3_gazebo.rviz'],
            output='screen')
    ])