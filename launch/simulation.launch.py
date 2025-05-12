# gazebo_simulation.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Launch arguments
    environment = LaunchConfiguration('environment', default='simple')
    robot1_model = LaunchConfiguration('robot1_model', default='waffle_pi')
    robot2_model = LaunchConfiguration('robot2_model', default='waffle_pi')
    num_tasks = LaunchConfiguration('num_tasks', default='10')
    scenario_type = LaunchConfiguration('scenario_type', default='automotive')
    
    # Environment world file based on selected environment
    world_file = LaunchConfiguration(
        'world_file',
        default=[FindPackageShare('decentralized_control'), '/worlds/', environment, '_environment.world'])
    
    # Standard ROS 2 launch description
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'environment',
            default_value='simple',
            description='Environment type: simple, cluttered, or assembly'),
            
        DeclareLaunchArgument(
            'robot1_model',
            default_value='waffle_pi',
            description='TurtleBot3 model for robot 1'),
            
        DeclareLaunchArgument(
            'robot2_model',
            default_value='waffle_pi',
            description='TurtleBot3 model for robot 2'),
            
        DeclareLaunchArgument(
            'num_tasks',
            default_value='10',
            description='Number of tasks to generate'),
            
        DeclareLaunchArgument(
            'scenario_type',
            default_value='automotive',
            description='Type of industrial scenario: automotive, electronics, or furniture'),
        
        # Launch Gazebo with the selected world
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_file
            ],
            output='screen'),
        
        # Configure Gazebo physics
        Node(
            package='decentralized_control',
            executable='gazebo_physics_configurator',
            name='gazebo_physics_configurator',
            output='screen'),
        
        # Spawn Robot 1 with manipulator
        Node(
            package='decentralized_control',
            executable='robot_model_spawner',
            name='robot_model_spawner1',
            parameters=[{
                'robot_id': 1,
                'robot_model': robot1_model,
                'manipulator_model': 'open_manipulator_x',
                'position': [1.0, 1.0, 0.0],
                'orientation': [0.0, 0.0, 0.0]
            }],
            output='screen'),
        
        # Spawn Robot 2 with manipulator
        Node(
            package='decentralized_control',
            executable='robot_model_spawner',
            name='robot_model_spawner2',
            parameters=[{
                'robot_id': 2,
                'robot_model': robot2_model,
                'manipulator_model': 'open_manipulator_x',
                'position': [3.0, 3.0, 0.0],
                'orientation': [0.0, 0.0, 3.1415]  # PI radians (facing opposite direction)
            }],
            output='screen'),
        
        # Robot 1 controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/robot1/controller_manager'],
            output='screen',
            namespace='robot1'),
            
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['manipulator_controller', '--controller-manager', '/robot1/controller_manager'],
            output='screen',
            namespace='robot1'),
            
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller', '--controller-manager', '/robot1/controller_manager'],
            output='screen',
            namespace='robot1'),
            
        # Robot 2 controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/robot2/controller_manager'],
            output='screen',
            namespace='robot2'),
            
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['manipulator_controller', '--controller-manager', '/robot2/controller_manager'],
            output='screen',
            namespace='robot2'),
            
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller', '--controller-manager', '/robot2/controller_manager'],
            output='screen',
            namespace='robot2'),
        
        # Assembly task visualizer
        Node(
            package='decentralized_control',
            executable='assembly_task_visualizer',
            name='assembly_task_visualizer',
            output='screen'),
        
        # Industrial scenario generator
        Node(
            package='decentralized_control',
            executable='industrial_scenario_generator',
            name='industrial_scenario_generator',
            parameters=[{
                'scenario_type': scenario_type,
                'complexity': 'medium',
                'environment_scale': 1.0,
                'output_dir': '/tmp/decentralized_control/scenarios'
            }],
            output='screen'),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('decentralized_control'),
                'config', 'mobile_manipulator.rviz'
            ])],
            output='screen')
    ])