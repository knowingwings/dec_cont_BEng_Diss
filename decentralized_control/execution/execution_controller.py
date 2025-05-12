#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from threading import Lock
import numpy as np
import time
import os
import json
from collections import defaultdict

from decentralized_control.msg import (
    Task, TaskList, TaskAssignment, RobotState,
    CollaborationRequest, CollaborationResponse, SynchronizationSignal
)

class MetricsCollector(Node):
    """
    Collects performance metrics during simulation runs.
    """
    
    def __init__(self):
        super().__init__('metrics_collector')
        
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
        for task in msg.tasks:
            task_id = task.id
            if task_id not in self.task_completion_times:
                self.task_completion_times[task_id] = None
    
    def assignment_callback(self, msg):
        """Process task assignment updates."""
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
        self.message_counts['bids'] += 1
        self.total_messages += 1
        
        # Increment auction iterations
        self.auction_iterations += 1
    
    def heartbeat_callback(self, msg):
        """Process heartbeat messages for metrics."""
        self.message_counts['heartbeats'] += 1
        self.total_messages += 1
    
    def robot_state_callback(self, msg):
        """Process robot state messages for metrics."""
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
            'message_counts': self.message_counts,
            'auction_iterations': self.auction_iterations,
            'task_reassignments': dict(self.task_reassignments),
            'recovery_events': self.recovery_events,
            'recovery_times': self.recovery_times
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

class ExecutionController(Node):
    """
    Controls the execution of assigned tasks and manages coordination between robots.
    """
    
    def __init__(self):
        super().__init__('execution_controller')
        
        # Initialize parameters
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('execution_timeout', 30.0)  # seconds
        self.declare_parameter('collaboration_timeout', 10.0)  # seconds
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.execution_timeout = self.get_parameter('execution_timeout').value
        self.collaboration_timeout = self.get_parameter('collaboration_timeout').value
        
        # Initialize member variables
        self.tasks = {}  # Dict of task_id -> Task
        self.assigned_tasks = []  # List of assigned Task objects
        self.current_task = None  # Currently executing task
        self.task_progress = 0.0  # Progress on current task (0-1)
        self.executing = False
        self.collaborating = False
        self.collaboration_partners = set()
        self.callback_group = ReentrantCallbackGroup()
        
        # Robot state
        self.position = np.zeros(3)  # [x, y, z]
        self.orientation = np.zeros(3)  # [roll, pitch, yaw]
        self.linear_vel = np.zeros(3)
        self.angular_vel = np.zeros(3)
        self.joint_positions = np.zeros(6)  # Assuming 6-DOF manipulator
        self.joint_velocities = np.zeros(6)
        self.capabilities = np.ones(5)  # Example capability vector
        self.failed = False
        
        # Locks for thread safety
        self.task_lock = Lock()  # For task-related state
        self.state_lock = Lock() # For robot state
        
        # Publishers
        self.state_publisher = self.create_publisher(
            RobotState,
            f'/robot{self.robot_id}/state',
            10)
            
        self.sync_publisher = self.create_publisher(
            SynchronizationSignal,
            f'/robot{self.robot_id}/sync',
            10)
            
        self.collab_request_publisher = self.create_publisher(
            CollaborationRequest,
            f'/robot{self.robot_id}/collaboration/request',
            10)
        
        # Subscribers
        self.task_subscriber = self.create_subscription(
            TaskList,
            '/tasks',
            self.task_callback,
            10,
            callback_group=self.callback_group)
            
        self.assignment_subscriber = self.create_subscription(
            TaskAssignment,
            '/auction/assignments',
            self.assignment_callback,
            10,
            callback_group=self.callback_group)
            
        self.collab_response_subscriber = self.create_subscription(
            CollaborationResponse,
            f'/robot{self.robot_id}/collaboration/response',
            self.collaboration_response_callback,
            10,
            callback_group=self.callback_group)
            
        # Other subscribers (for a full implementation)
        # self.odom_subscriber = self.create_subscription(...)
        # self.joint_state_subscriber = self.create_subscription(...)
        
        # Timers
        self.execution_timer = self.create_timer(
            0.1,  # 10 Hz
            self.execution_step,
            callback_group=self.callback_group)
            
        self.state_timer = self.create_timer(
            1.0,  # 1 Hz
            self.publish_state,
            callback_group=self.callback_group)
        
        self.get_logger().info(
            f'Execution controller initialized for robot {self.robot_id}')
    
    def task_callback(self, msg):
        """Process incoming task information"""
        with self.task_lock:
            for task in msg.tasks:
                self.tasks[task.id] = task
    
    def assignment_callback(self, msg):
        """Process task assignments"""
        with self.task_lock:
            # Clear current assignments that are no longer assigned to us
            current_assignments = [task.id for task in self.assigned_tasks]
            new_assignments = []
            
            for i in range(len(msg.task_ids)):
                task_id = msg.task_ids[i]
                robot_id = msg.robot_ids[i]
                
                if robot_id == self.robot_id and task_id in self.tasks:
                    # Add to assigned tasks if not already there
                    if task_id not in current_assignments:
                        new_assignments.append(self.tasks[task_id])
                        
                        # If task requires collaboration, send request
                        if self.tasks[task_id].requires_collaboration:
                            self.request_collaboration(task_id)
            
            # Update assigned tasks list
            self.assigned_tasks.extend(new_assignments)
    
    def request_collaboration(self, task_id):
        """Request collaboration from other robots if needed."""
        if not self.tasks[task_id].requires_collaboration:
            return
            
        request = CollaborationRequest()
        request.task_id = task_id
        request.robot_id = self.robot_id
        request.required_robots = self.tasks[task_id].required_robots
        request.timeout = self.collaboration_timeout
        
        self.collab_request_publisher.publish(request)
        self.collaborating = True
    
    def collaboration_response_callback(self, msg):
        """Handle responses to collaboration requests."""
        if not self.collaborating or msg.task_id not in [t.id for t in self.assigned_tasks]:
            return
            
        if msg.accepted:
            self.collaboration_partners.add(msg.robot_id)
            
            # Check if we have enough partners
            task = next(t for t in self.assigned_tasks if t.id == msg.task_id)
            if len(self.collaboration_partners) >= task.required_robots - 1:
                self.ready_for_execution(task.id)
    
    def ready_for_execution(self, task_id):
        """Signal that we're ready to execute a collaborative task."""
        sync_msg = SynchronizationSignal()
        sync_msg.task_id = task_id
        sync_msg.robot_id = self.robot_id
        sync_msg.partners = list(self.collaboration_partners)
        sync_msg.status = "READY"
        
        self.sync_publisher.publish(sync_msg)
        self.get_logger().info(f'Ready to execute task {task_id}')
    
    def execution_step(self):
        """Execute current task"""
        with self.task_lock:
            if self.executing or not self.assigned_tasks:
                return
                
            # Get next task
            self.current_task = self.assigned_tasks[0]
            
            # If collaborative task, wait for coordination
            if self.current_task.requires_collaboration and not self.collaborating:
                return
                
            self.executing = True
            self.task_progress = 0.0
            
            # Remove from assigned tasks list
            self.assigned_tasks = self.assigned_tasks[1:]
            
            self.get_logger().info(f'Starting execution of task {self.current_task.id}')
    
    def publish_state(self):
        """Publish robot state information"""
        with self.state_lock, self.task_lock:
            msg = RobotState()
            msg.robot_id = self.robot_id
            msg.position = self.position.tolist()
            msg.orientation = self.orientation.tolist()
            msg.capabilities = self.capabilities.tolist()
            
            # Calculate workload based on remaining tasks
            workload = 0.0
            for task in self.assigned_tasks:
                workload += task.execution_time
            if self.current_task:
                workload += self.current_task.execution_time * (1.0 - self.task_progress)
            msg.workload = workload
            
            msg.failed = self.failed
            msg.executing = self.executing
            msg.collaborating = self.collaborating
            
            if self.current_task:
                msg.current_task_id = self.current_task.id
            
            self.state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    executor = MultiThreadedExecutor()
    node = ExecutionController()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()