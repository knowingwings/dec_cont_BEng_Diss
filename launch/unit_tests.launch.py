# unit_tests.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Run the test nodes
        Node(
            package='decentralized_control',
            executable='test_communication',
            name='test_communication',
            output='screen'),
            
        Node(
            package='decentralized_control',
            executable='test_auction',
            name='test_auction',
            output='screen'),
            
        Node(
            package='decentralized_control',
            executable='test_consensus',
            name='test_consensus',
            output='screen'),
            
        Node(
            package='decentralized_control',
            executable='test_recovery',
            name='test_recovery',
            output='screen'),
            
        Node(
            package='decentralized_control',
            executable='test_task_manager',
            name='test_task_manager',
            output='screen'),
            
        Node(
            package='decentralized_control',
            executable='test_execution',
            name='test_execution',
            output='screen'),
            
        Node(
            package='decentralized_control',
            executable='test_metrics',
            name='test_metrics',
            output='screen'),
            
        Node(
            package='decentralized_control',
            executable='test_comparative',
            name='test_comparative',
            output='screen'),
    ])