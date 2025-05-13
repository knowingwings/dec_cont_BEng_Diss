#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from threading import Lock

from dec_control.msg import Heartbeat, RobotState, TaskAssignment

class RecoveryNode(Node):
    """
    Implements failure detection and recovery mechanisms.
    """
    
    def __init__(self):
        super().__init__('recovery_node')
        
        # Add lock for thread safety
        self.lock = Lock()
        
        # Get robot ID from parameter
        self.declare_parameter('robot_id', 1)
        self.robot_id = self.get_parameter('robot_id').value
        
        # Initialize recovery parameters
        self.declare_parameter('heartbeat_timeout', 3.0)  # Seconds
        self.declare_parameter('progress_monitoring_interval', 5.0)  # Seconds
        self.declare_parameter('progress_threshold', 0.05)  # Minimum progress rate
        
        # Get parameters
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self.progress_monitoring_interval = self.get_parameter('progress_monitoring_interval').value
        self.progress_threshold = self.get_parameter('progress_threshold').value
        
        # Initialize internal state variables
        self.robot_statuses = {}  # Robot ID -> status (0=normal, 1=suspected, 2=failed)
        self.last_heartbeat_time = {}  # Robot ID -> last heartbeat time
        self.last_progress_time = {}  # Task ID -> last progress update time
        self.progress_values = {}  # Task ID -> progress value
        self.recovery_mode = False
        self.recovery_start_time = 0
        self.recovery_complete_time = 0
        self.failed_robots = set()
        
        # Publishers
        self.heartbeat_publisher = self.create_publisher(
            Heartbeat, f'/robot{self.robot_id}/heartbeat', 10)
        
        # Subscribers
        self.heartbeat_subscribers = []
        for i in range(1, 3):  # For dual mobile manipulators
            if i != self.robot_id:
                self.heartbeat_subscribers.append(
                    self.create_subscription(
                        Heartbeat, f'/robot{i}/heartbeat', self.heartbeat_callback, 10))
        
        self.robot_state_subscribers = []
        for i in range(1, 3):  # For dual mobile manipulators
            if i != self.robot_id:
                self.robot_state_subscribers.append(
                    self.create_subscription(
                        RobotState, f'/robot{i}/state', self.robot_state_callback, 10))
        
        self.assignment_subscriber = self.create_subscription(
            TaskAssignment, '/auction/assignments', self.assignment_callback, 10)
        
        # Timer for monitoring
        self.monitor_timer = self.create_timer(
            1.0, self.monitor_callback)  # 1Hz monitoring
        
        # Timer for heartbeat
        self.heartbeat_timer = self.create_timer(
            1.0, self.send_heartbeat)  # 1Hz heartbeat
        
        self.get_logger().info(
            f'Recovery node initialized for Robot {self.robot_id}')
    
    def heartbeat_callback(self, msg):
        """Process heartbeat signals"""
        with self.lock:
            robot_id = msg.robot_id
            
            # Update last heartbeat time
            self.last_heartbeat_time[robot_id] = time.time()
            
            # If robot was suspected, clear suspicion
            if self.robot_statuses.get(robot_id, 0) == 1:
                self.robot_statuses[robot_id] = 0
                self.get_logger().info(f'Robot {robot_id} heartbeat resumed, clearing suspicion')
    
    def robot_state_callback(self, msg):
        """Process robot state updates"""
        with self.lock:
            robot_id = msg.id
            
            # If robot reports as failed
            if msg.failed:
                self.robot_statuses[robot_id] = 2  # Mark as failed
                self.failed_robots.add(robot_id)
                
                if not self.recovery_mode:
                    self.get_logger().info(f'Robot {robot_id} reported failure, initiating recovery')
                    self.initiate_recovery(robot_id)
    
    def assignment_callback(self, msg):
        """Monitor assignment changes to detect recovery completion"""
        with self.lock:
            if not self.recovery_mode:
                return
            
            # Check if all tasks have been reassigned
            all_reassigned = True
            for i in range(len(msg.task_ids)):
                task_id = msg.task_ids[i]
                robot_id = msg.robot_ids[i]
                
                # If task is assigned to a failed robot or marked for recovery
                if robot_id in self.failed_robots or robot_id == -1:
                    all_reassigned = False
                    break
            
            if all_reassigned:
                self.recovery_complete_time = time.time()
                recovery_duration = self.recovery_complete_time - self.recovery_start_time
                
                self.get_logger().info(
                    f'Recovery complete after {recovery_duration:.2f} seconds')
                self.recovery_mode = False
                
    def monitor_callback(self):
        """Monitor for failures based on heartbeats and progress"""
        with self.lock:
            current_time = time.time()
            
            # Check heartbeats
            for robot_id, last_time in self.last_heartbeat_time.items():
                # Skip already failed robots
                if self.robot_statuses.get(robot_id, 0) == 2:
                    continue
                
                # Check if heartbeat timeout exceeded
                timeout_duration = current_time - last_time
                if timeout_duration > self.heartbeat_timeout:
                    if self.robot_statuses.get(robot_id, 0) == 0:
                        # Mark as suspected
                        self.robot_statuses[robot_id] = 1
                        self.get_logger().warn(
                            f'Robot {robot_id} heartbeat timeout ({timeout_duration:.1f}s), marking as suspected')
                    elif self.robot_statuses.get(robot_id, 0) == 1:
                        # Mark as failed
                        self.robot_statuses[robot_id] = 2
                        self.failed_robots.add(robot_id)
                        
                        self.get_logger().error(
                            f'Robot {robot_id} confirmed failed, initiating recovery')
                        self.initiate_recovery(robot_id)
            
            # Monitor task progress if needed
            self.check_task_progress(current_time)
    
    def send_heartbeat(self):
        """Send heartbeat signal"""
        msg = Heartbeat()
        msg.robot_id = self.robot_id
        msg.timestamp = int(time.time() * 1000)  # milliseconds
        msg.status = 0  # 0 = Normal
        
        self.heartbeat_publisher.publish(msg)
    
    def initiate_recovery(self, failed_robot_id):
        """Initiate the recovery process"""
        with self.lock:
            self.recovery_mode = True
            self.recovery_start_time = time.time()
            
            self.get_logger().info(f'Initiating recovery for Robot {failed_robot_id}')
            
            # In a real implementation, this would communicate with the auction node
            # to trigger the recovery auction process
            # For now, we'll rely on the auction node to detect failure through RobotState messages
    
    def check_task_progress(self, current_time):
        """Monitor task progress and detect stalls"""
        with self.lock:
            # Skip if already in recovery
            if self.recovery_mode:
                return
            
            for task_id, last_time in list(self.last_progress_time.items()):
                # Skip completed tasks
                if self.progress_values.get(task_id, 0) >= 1.0:
                    continue
                
                # Check if progress monitoring interval exceeded
                if current_time - last_time > self.progress_monitoring_interval:
                    # In a real implementation, we would check actual progress here
                    # For now, just remove from monitoring to avoid false positives
                    self.last_progress_time.pop(task_id, None)