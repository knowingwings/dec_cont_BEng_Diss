# metrics_collector.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import json
import os
import numpy as np
from collections import defaultdict
from threading import Lock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from pathlib import Path

from decentralized_control.msg import Task, TaskList, TaskAssignment, Bid, RobotState, Heartbeat, NetworkTopology
from std_msgs.msg import Float64MultiArray

class MetricsCollector(Node):
    """
    Collects performance metrics during simulation runs.
    """
    
    def __init__(self):
        super().__init__('metrics_collector')
        
        # Add lock for thread safety
        self.lock = Lock()
        
        # Initialize parameters
        self.declare_parameter('output_dir', '/tmp/decentralized_control/results')
        self.declare_parameter('experiment_id', time.strftime('%Y%m%d_%H%M%S'))
        self.declare_parameter('num_tasks', 10)
        self.declare_parameter('delay_ms', 0)
        self.declare_parameter('packet_loss', 0.0)
        self.declare_parameter('epsilon', 0.05)
        
        # Get parameters
        self.output_dir = self.get_parameter('output_dir').value
        self.experiment_id = self.get_parameter('experiment_id').value
        self.num_tasks = self.get_parameter('num_tasks').value
        self.delay_ms = self.get_parameter('delay_ms').value
        self.packet_loss = self.get_parameter('packet_loss').value
        self.epsilon = self.get_parameter('epsilon').value
        
        # Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Initialize metrics
        self.start_time = time.time()
        self.task_completion_times = {}
        self.task_assignments = {}
        self.task_final_assignments = {}
        self.task_reassignments = defaultdict(int)
        self.auction_iterations = 0
        self.total_messages = 0
        self.message_counts = {
            'bids': 0,
            'assignments': 0,
            'heartbeats': 0,
            'state_updates': 0
        }
        self.convergence_time = None
        self.makespan = None
        self.recovery_events = []
        self.recovery_times = []
        
        # Set up subscribers
        self.task_list_sub = self.create_subscription(
            TaskList, '/tasks', self.task_list_callback, 10)
        
        self.assignment_sub = self.create_subscription(
            TaskAssignment, '/auction/assignments', self.assignment_callback, 10)
        
        self.bid_subs = []
        for i in range(1, 3):  # For dual robots
            self.bid_subs.append(
                self.create_subscription(
                    Bid, f'/robot{i}/auction/bids', self.bid_callback, 10))
        
        self.heartbeat_subs = []
        for i in range(1, 3):  # For dual robots
            self.heartbeat_subs.append(
                self.create_subscription(
                    Heartbeat, f'/robot{i}/heartbeat', self.heartbeat_callback, 10))
        
        self.robot_state_subs = []
        for i in range(1, 3):  # For dual robots
            self.robot_state_subs.append(
                self.create_subscription(
                    RobotState, f'/robot{i}/state', self.robot_state_callback, 10))
        
        # Timer for periodic metrics calculation and saving
        self.metrics_timer = self.create_timer(5.0, self.calculate_metrics)
        
        self.get_logger().info(
            f'Metrics collector initialized for experiment {self.experiment_id}')
    
    def task_list_callback(self, msg):
        """Process task list updates."""
        with self.lock:
            for task in msg.tasks:
                task_id = task.id
                if task_id not in self.task_completion_times:
                    self.task_completion_times[task_id] = None
    
    def assignment_callback(self, msg):
        """Process task assignment updates."""
        with self.lock:
            self.message_counts['assignments'] += 1
            self.total_messages += 1
            
            timestamp = time.time()
            
            # Check for assignment changes
            for i in range(len(msg.task_ids)):
                task_id = msg.task_ids[i]
                robot_id = msg.robot_ids[i]
                
                # Track assignment changes
                if task_id in self.task_assignments and self.task_assignments[task_id] != robot_id:
                    self.task_reassignments[task_id] += 1
                
                # Update current assignment
                self.task_assignments[task_id] = robot_id
                
                # If this is a new final assignment, record it
                if robot_id > 0 and task_id not in self.task_final_assignments:
                    self.task_final_assignments[task_id] = (robot_id, timestamp)
                    
                    # Check if all tasks are assigned (convergence)
                    if len(self.task_final_assignments) == self.num_tasks and self.convergence_time is None:
                        self.convergence_time = timestamp - self.start_time
                        self.get_logger().info(f'Convergence achieved in {self.convergence_time:.2f} seconds')
            
            # Check if all tasks are completed
            completed_tasks = sum(1 for robot_id in self.task_assignments.values() if robot_id > 0)
            if completed_tasks == self.num_tasks and self.makespan is None:
                self.makespan = timestamp - self.start_time
                self.get_logger().info(f'All tasks assigned, makespan: {self.makespan:.2f} seconds')
    
    def bid_callback(self, msg):
        """Process bid messages for metrics."""
        with self.lock:
            self.message_counts['bids'] += 1
            self.total_messages += 1
            self.auction_iterations += 1
    
    def heartbeat_callback(self, msg):
        """Process heartbeat messages for metrics."""
        with self.lock:
            self.message_counts['heartbeats'] += 1
            self.total_messages += 1
    
    def robot_state_callback(self, msg):
        """Process robot state messages for metrics."""
        with self.lock:
            self.message_counts['state_updates'] += 1
            self.total_messages += 1
            
            # Detect robot failures and recovery
            if hasattr(msg, 'failed') and msg.failed:
                timestamp = time.time()
                robot_id = msg.id
                
                # Record failure event
                self.recovery_events.append({
                    'event': 'failure',
                    'robot_id': robot_id,
                    'timestamp': timestamp
                })
    
    def calculate_metrics(self):
        """Periodically calculate metrics and save to file."""
        with self.lock:
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            
            metrics = {
                'experiment_id': self.experiment_id,
                'parameters': {
                    'num_tasks': self.num_tasks,
                    'delay_ms': self.delay_ms,
                    'packet_loss': self.packet_loss,
                    'epsilon': self.epsilon
                },
                'elapsed_time': elapsed_time,
                'convergence_time': self.convergence_time,
                'makespan': self.makespan,
                'total_messages': self.total_messages,
                'message_counts': self.message_counts.copy(),
                'auction_iterations': self.auction_iterations,
                'task_reassignments': dict(self.task_reassignments),
                'recovery_events': self.recovery_events.copy(),
                'recovery_times': self.recovery_times.copy()
            }
            
            # Calculate averages
            avg_reassignments = sum(self.task_reassignments.values()) / max(1, len(self.task_reassignments))
            metrics['avg_reassignments_per_task'] = avg_reassignments
            
            # Save metrics to file
            filename = os.path.join(
                self.output_dir, 
                f'metrics_{self.experiment_id}_T{self.num_tasks}_D{self.delay_ms}_'
                f'P{int(self.packet_loss*100)}_E{int(self.epsilon*100)}.json')
            
            with open(filename, 'w') as f:
                json.dump(metrics, f, indent=2)
            
            self.get_logger().info(f'Metrics saved to {filename}')

class NetworkMetricsCollector(Node):
    """Collects and records network performance metrics."""
    
    def __init__(self):
        super().__init__('network_metrics_collector')
        
        # QoS profile for reliable message delivery
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize storage
        self.metrics_lock = Lock()
        self.network_metrics = defaultdict(lambda: {
            'latencies': [],
            'message_rates': [],
            'packet_losses': [],
            'timestamps': []
        })
        
        self.consensus_errors = []
        self.gamma_values = defaultdict(list)
        self.timestamps = []
        
        # Subscribe to network metrics
        self.create_subscription(
            Float64MultiArray,
            'network_latency',
            self.latency_callback,
            qos_profile
        )
        
        self.create_subscription(
            Float64MultiArray,
            'message_rate',
            self.message_rate_callback,
            qos_profile
        )
        
        self.create_subscription(
            Float64MultiArray,
            'packet_loss',
            self.packet_loss_callback,
            qos_profile
        )
        
        # Subscribe to consensus metrics
        self.create_subscription(
            Float64MultiArray,
            'consensus_error',
            self.consensus_error_callback,
            qos_profile
        )
        
        self.create_subscription(
            Float64MultiArray,
            'adaptation_params',
            self.adaptation_callback,
            qos_profile
        )
        
        # Parameters
        self.declare_parameter('output_dir', 'results/consensus')
        self.declare_parameter('experiment_id', 'default')
        self.declare_parameter('sampling_rate', 10.0)
        self.declare_parameter('history_length', 1000)
        
        # Timer for periodic data recording
        period = 1.0 / self.get_parameter('sampling_rate').value
        self.timer = self.create_timer(period, self.record_data)
        
        self.get_logger().info('Network metrics collector initialized')
    
    def latency_callback(self, msg):
        """Record network latency measurements."""
        with self.metrics_lock:
            robot_id = int(msg.data[0])
            latency = float(msg.data[1])
            self.network_metrics[robot_id]['latencies'].append(latency)
    
    def message_rate_callback(self, msg):
        """Record message rate measurements."""
        with self.metrics_lock:
            robot_id = int(msg.data[0])
            rate = float(msg.data[1])
            self.network_metrics[robot_id]['message_rates'].append(rate)
    
    def packet_loss_callback(self, msg):
        """Record packet loss measurements."""
        with self.metrics_lock:
            robot_id = int(msg.data[0])
            loss_rate = float(msg.data[1])
            self.network_metrics[robot_id]['packet_losses'].append(loss_rate)
    
    def consensus_error_callback(self, msg):
        """Record consensus error measurements."""
        with self.metrics_lock:
            error = float(msg.data[0])
            self.consensus_errors.append(error)
    
    def adaptation_callback(self, msg):
        """Record adaptation parameter values."""
        with self.metrics_lock:
            robot_id = int(msg.data[0])
            gamma = float(msg.data[1])
            self.gamma_values[robot_id].append(gamma)
    
    def record_data(self):
        """Periodically record timestamped data."""
        current_time = self.get_clock().now().to_msg().sec
        
        with self.metrics_lock:
            self.timestamps.append(current_time)
            
            # Trim data if exceeding history length
            history_length = self.get_parameter('history_length').value
            
            if len(self.timestamps) > history_length:
                self.timestamps = self.timestamps[-history_length:]
                self.consensus_errors = self.consensus_errors[-history_length:]
                
                for robot_id in self.network_metrics:
                    for metric_type in ['latencies', 'message_rates', 'packet_losses']:
                        self.network_metrics[robot_id][metric_type] = \
                            self.network_metrics[robot_id][metric_type][-history_length:]
                
                for robot_id in self.gamma_values:
                    self.gamma_values[robot_id] = \
                        self.gamma_values[robot_id][-history_length:]
    
    def get_consensus_errors(self):
        """Get recorded consensus errors."""
        with self.metrics_lock:
            return self.consensus_errors.copy()
    
    def get_message_rates(self):
        """Get recorded message rates."""
        with self.metrics_lock:
            rates = []
            for robot_id in self.network_metrics:
                rates.extend(self.network_metrics[robot_id]['message_rates'])
            return rates
    
    def save_metrics(self):
        """Save collected metrics to file."""
        output_dir = Path(self.get_parameter('output_dir').value)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        experiment_id = self.get_parameter('experiment_id').value
        output_file = output_dir / f'network_metrics_{experiment_id}.json'
        
        with self.metrics_lock:
            metrics_data = {
                'timestamps': self.timestamps,
                'consensus_errors': self.consensus_errors,
                'network_metrics': dict(self.network_metrics),
                'gamma_values': dict(self.gamma_values)
            }
        
        with open(output_file, 'w') as f:
            json.dump(metrics_data, f)
        
        self.get_logger().info(f'Metrics saved to {output_file}')
    
    def configure(self, **params):
        """Configure the metrics collector with new parameters."""
        for param_name, value in params.items():
            self.set_parameter(rclpy.Parameter(param_name, value=value))
        
        # Update timer period if sampling rate changed
        if 'sampling_rate' in params:
            self.timer.cancel()
            period = 1.0 / params['sampling_rate']
            self.timer = self.create_timer(period, self.record_data)
        
        self.get_logger().info('Metrics collector configuration updated')

def main(args=None):
    rclpy.init(args=args)
    collector = NetworkMetricsCollector()
    rclpy.spin(collector)
    collector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()