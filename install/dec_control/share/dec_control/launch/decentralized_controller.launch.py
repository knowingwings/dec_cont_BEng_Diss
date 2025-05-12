# decentralized_controller.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
import yaml
import os

def generate_launch_description():
    """Generate launch description for the decentralized control system with experimental parameters"""
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='config/experiments.yaml',
        description='Path to experiment configuration file'
    )
    
    network_scenario_arg = DeclareLaunchArgument(
        'network_scenario',
        default_value='ideal',
        description='Network scenario to simulate'
    )
    
    algorithm_variant_arg = DeclareLaunchArgument(
        'algorithm_variant',
        default_value='network_aware',
        description='Consensus algorithm variant to use'
    )
    
    initial_state_arg = DeclareLaunchArgument(
        'initial_state',
        default_value='divergent_position',
        description='Initial state configuration'
    )
    
    experiment_set_arg = DeclareLaunchArgument(
        'experiment_set',
        default_value='network_robustness',
        description='Experiment set to run'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='results/consensus',
        description='Directory for experiment results'
    )
    
    return LaunchDescription([
        config_file_arg,
        network_scenario_arg,
        algorithm_variant_arg,
        initial_state_arg,
        experiment_set_arg,
        output_dir_arg,
        OpaqueFunction(function=launch_consensus_nodes)
    ])

def launch_consensus_nodes(context):
    """Launch nodes based on configuration file and arguments."""
    # Get launch configurations
    config_file = LaunchConfiguration('config_file').perform(context)
    network_scenario = LaunchConfiguration('network_scenario').perform(context)
    algorithm_variant = LaunchConfiguration('algorithm_variant').perform(context)
    initial_state = LaunchConfiguration('initial_state').perform(context)
    experiment_set = LaunchConfiguration('experiment_set').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    
    # Load configuration
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # Get configurations
    default_params = config['consensus_experiments']['default_params']
    network_config = next(
        s for s in config['consensus_experiments']['network_scenarios']
        if s['name'] == network_scenario
    )
    algorithm_config = next(
        a for a in config['consensus_experiments']['algorithm_variants']
        if a['name'] == algorithm_variant
    )
    state_config = next(
        s for s in config['consensus_experiments']['initial_states']
        if s['name'] == initial_state
    )
    
    nodes = []
    
    # Launch network monitor
    nodes.append(Node(
        package='decentralized_control',
        executable='network_monitor',
        name='network_monitor',
        parameters=[{
            'num_robots': default_params['num_robots'],
            'update_period': 0.1,
            'quality_threshold': 0.3,
            'latency_window': 10
        }]
    ))
    
    # Launch communication middleware with network scenario configuration
    middleware_params = {
        'delay_mean_ms': network_config.get('delay_mean_ms', 0),
        'delay_stddev_ms': network_config.get('delay_stddev_ms', 0),
        'packet_loss_prob': network_config.get('packet_loss_prob', 0.0),
        'delay_pattern': network_config.get('delay_pattern', 'constant'),
        'base_delay_ms': network_config.get('base_delay_ms', 0),
        'amplitude_ms': network_config.get('amplitude_ms', 0),
        'period_s': network_config.get('period_s', 10.0)
    }
    
    nodes.append(Node(
        package='decentralized_control',
        executable='communication_middleware',
        name='communication_middleware',
        parameters=[middleware_params]
    ))
    
    # Launch metrics collectors
    nodes.append(Node(
        package='decentralized_control',
        executable='metrics_collector',
        name='metrics_collector',
        parameters=[{
            'output_dir': output_dir,
            'experiment_id': f"{network_scenario}_{algorithm_variant}_{initial_state}",
            'sampling_rate': 10.0,
            'history_length': 1000
        }]
    ))
    
    nodes.append(Node(
        package='decentralized_control',
        executable='network_metrics_collector',
        name='network_metrics_collector',
        parameters=[{
            'output_dir': output_dir,
            'sampling_rate': 10.0,
            'history_length': 1000
        }]
    ))
    
    # Launch consensus nodes
    for i in range(default_params['num_robots']):
        robot_id = i + 1
        robot_config = state_config[f'robot{robot_id}']
        
        # Combine algorithm and robot-specific parameters
        consensus_params = {
            'robot_id': robot_id,
            'min_update_period': default_params['min_update_period'],
            'max_delay': default_params['max_delay']
        }
        consensus_params.update(algorithm_config)
        
        # Add initial state
        consensus_params.update({
            'initial_position': robot_config['position'],
            'initial_orientation': robot_config['orientation'],
            'initial_capabilities': robot_config['capabilities'],
            'initial_workload': robot_config['workload']
        })
        
        nodes.append(Node(
            package='decentralized_control',
            executable='time_varying_consensus_node',
            name=f'consensus_node_{robot_id}',
            parameters=[consensus_params]
        ))
    
    return nodes