#!/usr/bin/env python3

import rclpy
import yaml
import argparse
import os
from threading import Thread
import time
import signal
import sys
from datetime import datetime

from decentralized_control.consensus.time_varying_consensus import TimeVaryingConsensusNode
from decentralized_control.communication_middleware import CommunicationMiddleware
from decentralized_control.metrics_collector import MetricsCollector, NetworkMetricsCollector

class ExperimentManager:
    """Manages the execution of consensus experiments with different configurations."""
    
    def __init__(self, config_file):
        # Load experiment configuration
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Create output directory
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.base_output_dir = os.path.join(
            self.config['consensus_experiments']['default_params']['output_dir'],
            f'experiment_run_{timestamp}'
        )
        os.makedirs(self.base_output_dir, exist_ok=True)
        
        # Save configuration copy
        config_copy = os.path.join(self.base_output_dir, 'experiment_config.yaml')
        with open(config_copy, 'w') as f:
            yaml.dump(self.config, f)
        
        # Initialize ROS
        rclpy.init()
        
        # Signal handling
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.running = True
        self.current_experiment = None
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signal."""
        print("\nShutdown requested...")
        self.running = False
        if self.current_experiment:
            self.current_experiment.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    def run_experiments(self):
        """Run all experiment combinations."""
        default_params = self.config['consensus_experiments']['default_params']
        
        # Run each experiment set
        for set_name, set_config in self.config['consensus_experiments']['experiment_sets'].items():
            print(f"\nRunning experiment set: {set_name}")
            print(f"Description: {set_config['description']}")
            
            # Create output directory for this set
            set_output_dir = os.path.join(self.base_output_dir, set_name)
            os.makedirs(set_output_dir, exist_ok=True)
            
            # Run all combinations
            for network in set_config['network_scenarios']:
                for algorithm in set_config['algorithm_variants']:
                    for initial_state in set_config['initial_states']:
                        if not self.running:
                            return
                        
                        # Configure experiment
                        experiment = ExperimentRunner(
                            network_scenario=network,
                            algorithm_variant=algorithm,
                            initial_state=initial_state,
                            output_dir=set_output_dir,
                            config=self.config,
                            num_runs=set_config['runs_per_combination']
                        )
                        
                        self.current_experiment = experiment
                        experiment.run()
                        
                        # Wait between experiments
                        time.sleep(2.0)

class ExperimentRunner:
    """Runs a single experiment with specific configuration."""
    
    def __init__(self, network_scenario, algorithm_variant, initial_state, 
                 output_dir, config, num_runs):
        self.network_scenario = network_scenario
        self.algorithm_variant = algorithm_variant
        self.initial_state = initial_state
        self.output_dir = output_dir
        self.config = config
        self.num_runs = num_runs
        
        self.nodes = []
        self.node_threads = []
    
    def setup_experiment(self, run_number):
        """Set up nodes for a single experiment run."""
        # Create unique experiment ID
        experiment_id = (
            f"{self.network_scenario}_{self.algorithm_variant}_"
            f"{self.initial_state}_run{run_number}"
        )
        
        # Get configurations
        network_config = next(
            s for s in self.config['consensus_experiments']['network_scenarios']
            if s['name'] == self.network_scenario
        )
        algorithm_config = next(
            a for a in self.config['consensus_experiments']['algorithm_variants']
            if a['name'] == self.algorithm_variant
        )
        state_config = next(
            s for s in self.config['consensus_experiments']['initial_states']
            if s['name'] == self.initial_state
        )
        
        # Create communication middleware
        middleware = CommunicationMiddleware()
        middleware.delay_mean = network_config['delay_mean_ms'] / 1000.0
        middleware.delay_stddev = network_config.get('delay_stddev_ms', 0) / 1000.0
        middleware.packet_loss_prob = network_config.get('packet_loss_prob', 0.0)
        self.nodes.append(middleware)
        
        # Create consensus nodes
        for i in range(2):  # For dual robot system
            node = TimeVaryingConsensusNode()
            node.robot_id = i + 1
            
            # Configure consensus parameters
            for param, value in algorithm_config.items():
                if hasattr(node, param):
                    setattr(node, param, value)
            
            # Set initial state
            robot_config = state_config[f'robot{i+1}']
            node.state_vector = np.concatenate([
                robot_config['position'],
                robot_config['orientation'],
                robot_config['capabilities'],
                [robot_config['workload'], 0.0]
            ])
            
            self.nodes.append(node)
        
        # Create metrics collectors
        metrics = MetricsCollector()
        metrics.experiment_id = experiment_id
        metrics.output_dir = os.path.join(self.output_dir, experiment_id)
        self.nodes.append(metrics)
        
        network_metrics = NetworkMetricsCollector()
        network_metrics.output_dir = os.path.join(self.output_dir, experiment_id)
        self.nodes.append(network_metrics)
        
        # Start all nodes
        self.node_threads = []
        for node in self.nodes:
            thread = Thread(target=lambda: rclpy.spin(node))
            thread.daemon = True
            thread.start()
            self.node_threads.append(thread)
        
        return experiment_id
    
    def run(self):
        """Run all iterations of this experiment configuration."""
        for run in range(self.num_runs):
            print(f"\nStarting experiment: {self.network_scenario}, "
                  f"{self.algorithm_variant}, {self.initial_state}, "
                  f"Run {run+1}/{self.num_runs}")
            
            # Set up experiment
            experiment_id = self.setup_experiment(run + 1)
            
            # Run for specified duration
            duration = self.config['consensus_experiments']['default_params']['run_duration']
            time.sleep(duration)
            
            # Save metrics
            for node in self.nodes:
                if hasattr(node, 'save_metrics'):
                    node.save_metrics(experiment_id)
            
            # Clean up
            self.shutdown()
            
            print(f"Completed run {run+1}")
    
    def shutdown(self):
        """Clean up experiment nodes."""
        for node in self.nodes:
            node.destroy_node()
        
        self.nodes.clear()
        self.node_threads.clear()

def main():
    parser = argparse.ArgumentParser(description='Run consensus experiments')
    parser.add_argument('--config', default='config/experiments.yaml',
                      help='Path to experiment configuration file')
    
    args = parser.parse_args()
    
    manager = ExperimentManager(args.config)
    manager.run_experiments()

if __name__ == '__main__':
    main()