#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time
import json
import os
from threading import Thread

from decentralized_control.communication_middleware import CommunicationMiddleware
from decentralized_control.metrics_collector import MetricsCollector
from decentralized_control.consensus.time_varying_consensus import TimeVaryingConsensusNode
from decentralized_control.msg import NetworkTopology, RobotState

class ExperimentRunner(Node):
    """
    Runs experiments to evaluate distributed consensus under various network conditions.
    """
    
    def __init__(self):
        super().__init__('experiment_runner')
        
        # Load experiment configuration
        self.declare_parameter('config_file', 'config/experiments.yaml')
        config_file = self.get_parameter('config_file').value
        
        # Ensure output directory exists
        self.output_dir = os.path.join(os.getcwd(), 'results')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Initialize network conditions
        self.network_conditions = [
            # Format: (delay_mean_ms, delay_stddev_ms, packet_loss_prob, description)
            (0, 0, 0.0, "ideal"),
            (50, 10, 0.0, "moderate_delay"),
            (100, 20, 0.1, "high_delay_loss"),
            (200, 50, 0.2, "severe_conditions")
        ]
        
        # Initialize experiment parameters
        self.declare_parameter('num_runs', 5)
        self.declare_parameter('run_duration', 60.0)  # seconds
        self.num_runs = self.get_parameter('num_runs').value
        self.run_duration = self.get_parameter('run_duration').value
        
        self.get_logger().info('Experiment runner initialized')
    
    def run_experiments(self):
        """Run experiments with different network conditions."""
        results = {}
        
        for delay_mean, delay_stddev, loss_prob, condition in self.network_conditions:
            condition_results = []
            
            for run in range(self.num_runs):
                self.get_logger().info(
                    f'Starting run {run+1}/{self.num_runs} for condition: {condition}')
                
                # Configure network middleware
                middleware = CommunicationMiddleware()
                middleware.delay_mean = delay_mean / 1000.0  # Convert to seconds
                middleware.delay_stddev = delay_stddev / 1000.0
                middleware.packet_loss_prob = loss_prob
                
                # Initialize metrics collector
                metrics = MetricsCollector()
                metrics.experiment_id = f'{condition}_run{run+1}'
                metrics.output_dir = self.output_dir
                
                # Create consensus nodes
                nodes = []
                for i in range(2):  # For dual robot system
                    node = TimeVaryingConsensusNode()
                    node.robot_id = i + 1
                    nodes.append(node)
                
                # Start experiment
                experiment_thread = Thread(target=self.run_single_experiment,
                                        args=(nodes, middleware, metrics))
                experiment_thread.start()
                experiment_thread.join(timeout=self.run_duration)
                
                # Collect results
                run_results = {
                    'convergence_time': metrics.convergence_time,
                    'total_messages': metrics.total_messages,
                    'message_counts': metrics.message_counts,
                    'network_conditions': {
                        'delay_mean_ms': delay_mean,
                        'delay_stddev_ms': delay_stddev,
                        'packet_loss_prob': loss_prob
                    }
                }
                condition_results.append(run_results)
                
                # Clean up
                for node in nodes:
                    node.destroy_node()
                middleware.destroy_node()
                metrics.destroy_node()
            
            # Calculate averages for this condition
            avg_results = self.calculate_average_results(condition_results)
            results[condition] = {
                'average': avg_results,
                'individual_runs': condition_results
            }
        
        # Save overall results
        self.save_results(results)
    
    def run_single_experiment(self, nodes, middleware, metrics):
        """Run a single experiment with given nodes and conditions."""
        try:
            # Set up initial states
            initial_states = [
                np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0]),
                np.array([10.0, 10.0, 10.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0])
            ]
            
            for i, node in enumerate(nodes):
                node.state_vector = initial_states[i]
                
                # Publish initial state
                msg = RobotState()
                msg.id = node.robot_id
                msg.position = initial_states[i][0:3].tolist()
                msg.orientation = initial_states[i][3:6].tolist()
                msg.capabilities = initial_states[i][6:11].tolist()
                msg.workload = float(initial_states[i][11])
                msg.failed = bool(initial_states[i][12] > 0.5)
                msg.timestamp = int(time.time() * 1000)
                
                middleware.simulate_constraints(
                    node.state_publisher, msg, RobotState)
            
            # Run for specified duration
            start_time = time.time()
            while time.time() - start_time < self.run_duration:
                rclpy.spin_once(middleware, timeout_sec=0.1)
                for node in nodes:
                    rclpy.spin_once(node, timeout_sec=0.1)
                rclpy.spin_once(metrics, timeout_sec=0.1)
                time.sleep(0.01)
                
        except Exception as e:
            self.get_logger().error(f'Error in experiment: {str(e)}')
    
    def calculate_average_results(self, condition_results):
        """Calculate average metrics across runs."""
        avg_results = {
            'convergence_time': np.mean([r['convergence_time'] for r in condition_results 
                                       if r['convergence_time'] is not None]),
            'total_messages': np.mean([r['total_messages'] for r in condition_results]),
            'message_counts': {}
        }
        
        # Average message counts by type
        msg_types = condition_results[0]['message_counts'].keys()
        for msg_type in msg_types:
            avg_results['message_counts'][msg_type] = np.mean(
                [r['message_counts'][msg_type] for r in condition_results])
        
        return avg_results
    
    def save_results(self, results):
        """Save experiment results to file."""
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.output_dir, f'consensus_results_{timestamp}.json')
        
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        
        self.get_logger().info(f'Results saved to {filename}')

def main(args=None):
    rclpy.init(args=args)
    runner = ExperimentRunner()
    runner.run_experiments()
    rclpy.shutdown()

if __name__ == '__main__':
    main()