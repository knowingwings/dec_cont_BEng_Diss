# decentralized_control/recovery/partial_failure_recovery.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time
from threading import Lock

from dec_control.msg import RobotState, TaskAssignment, CapabilityUpdate, RecoveryStatus

class PartialFailureRecoveryNode(Node):
    """
    Implements detection and recovery for partial robot failures
    by monitoring capability degradation and task feasibility.
    """
    
    def __init__(self):
        super().__init__('partial_failure_recovery_node')
        
        # Parameters
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('capability_threshold', 0.3)  # Minimum capability level
        self.declare_parameter('degradation_detection_threshold', 0.2)  # Significant degradation
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.capability_threshold = self.get_parameter('capability_threshold').value
        self.degradation_threshold = self.get_parameter('degradation_detection_threshold').value
        
        # Initialize state
        self.robot_capabilities = {}  # robot_id -> capability vector
        self.robot_capabilities_history = {}  # robot_id -> list of recent capability vectors
        self.history_length = 5  # Number of capability vectors to keep in history
        self.tasks = {}  # task_id -> Task
        self.task_assignments = {}  # task_id -> robot_id
        self.recovery_active = False
        self.lock = Lock()
        
        # Publishers
        self.capability_update_pub = self.create_publisher(
            CapabilityUpdate, f'/robot{self.robot_id}/capability_update', 10)
        
        self.recovery_status_pub = self.create_publisher(
            RecoveryStatus, '/recovery/status', 10)
        
        # Subscribers
        self.state_subscribers = []
        for i in range(1, 3):  # For dual mobile manipulators
            self.state_subscribers.append(
                self.create_subscription(
                    RobotState, f'/robot{i}/state', self.state_callback, 10))
        
        self.task_assignment_sub = self.create_subscription(
            TaskAssignment, '/auction/assignments', self.assignment_callback, 10)
            
        self.capability_update_sub = self.create_subscription(
            CapabilityUpdate, f'/robot{3-self.robot_id}/capability_update',  # Other robot
            self.capability_update_callback, 10)
        
        # Timer for monitoring and recovery
        self.monitor_timer = self.create_timer(1.0, self.monitor_capabilities)
        
        self.get_logger().info(
            f'Partial failure recovery node initialized for Robot {self.robot_id}')
    
    def state_callback(self, msg):
        """Process robot state updates to monitor capabilities."""
        with self.lock:
            robot_id = msg.id
            
            # Store capability vector
            self.robot_capabilities[robot_id] = np.array(msg.capabilities)
            
            # Update capability history
            if robot_id not in self.robot_capabilities_history:
                self.robot_capabilities_history[robot_id] = []
            
            history = self.robot_capabilities_history[robot_id]
            history.append(np.array(msg.capabilities))
            
            # Limit history length
            if len(history) > self.history_length:
                history.pop(0)
    
    def assignment_callback(self, msg):
        """Process task assignments."""
        with self.lock:
            # Update task assignments
            for i in range(len(msg.task_ids)):
                task_id = msg.task_ids[i]
                robot_id = msg.robot_ids[i]
                
                self.task_assignments[task_id] = robot_id
    
    def capability_update_callback(self, msg):
        """Process capability updates from other robots."""
        with self.lock:
            robot_id = msg.robot_id
            
            # Update capability vector with degradation
            self.robot_capabilities[robot_id] = np.array(msg.capabilities)
            
            # If in recovery mode, evaluate task feasibility
            if msg.in_recovery and not self.recovery_active:
                self.recovery_active = True
                
                # Evaluate task feasibility for tasks assigned to the other robot
                self.evaluate_task_feasibility(robot_id)
    
    def monitor_capabilities(self):
        """Monitor robot capabilities to detect degradation."""
        with self.lock:
            # Skip if not enough history
            if self.robot_id not in self.robot_capabilities_history:
                return
                
            history = self.robot_capabilities_history[self.robot_id]
            if len(history) < 2:
                return
            
            # Compare current capabilities with previous capabilities
            current_capabilities = history[-1]
            previous_capabilities = history[0]
            
            # Calculate degradation
            degradation = previous_capabilities - current_capabilities
            
            # Check for significant degradation
            significant_degradation = False
            degradation_mask = np.zeros_like(current_capabilities)
            
            for i in range(len(degradation)):
                if degradation[i] > self.degradation_threshold:
                    significant_degradation = True
                    degradation_mask[i] = 1
            
            if significant_degradation:
                # Create capability update message with degradation
                self.publish_capability_update(current_capabilities, degradation_mask)
                
                # Enter recovery mode
                self.initiate_partial_recovery(degradation_mask)
    
    def publish_capability_update(self, capabilities, degradation_mask):
        """Publish capability update message."""
        msg = CapabilityUpdate()
        msg.robot_id = self.robot_id
        msg.capabilities = capabilities.tolist()
        msg.degradation_mask = degradation_mask.tolist()
        msg.in_recovery = True
        msg.timestamp = int(time.time() * 1000)
        
        self.capability_update_pub.publish(msg)
        
        self.get_logger().info(
            f'Published capability update with degradation: {degradation_mask}')
    
    def initiate_partial_recovery(self, degradation_mask):
        """Initiate partial recovery for degraded capabilities."""
        self.recovery_active = True
        
        # Create recovery status message
        msg = RecoveryStatus()
        msg.robot_id = self.robot_id
        msg.recovery_type = "PARTIAL"
        msg.degraded_capabilities = degradation_mask.tolist()
        msg.timestamp = int(time.time() * 1000)
        
        self.recovery_status_pub.publish(msg)
        
        self.get_logger().info(
            f'Initiated partial recovery for Robot {self.robot_id}')
        
        # Evaluate task feasibility for assigned tasks
        self.evaluate_task_feasibility(self.robot_id)
    
    def evaluate_task_feasibility(self, robot_id):
        """
        Evaluate if tasks assigned to a robot are still feasible
        with degraded capabilities.
        """
        with self.lock:
            # Find tasks assigned to the robot
            assigned_tasks = []
            for task_id, assigned_robot in self.task_assignments.items():
                if assigned_robot == robot_id and task_id in self.tasks:
                    assigned_tasks.append(task_id)
            
            if not assigned_tasks:
                self.get_logger().info(f'No tasks assigned to Robot {robot_id}')
                return
            
            # Get robot capabilities
            if robot_id not in self.robot_capabilities:
                self.get_logger().warn(f'No capability data for Robot {robot_id}')
                return
            
            capabilities = self.robot_capabilities[robot_id]
            
            # Evaluate feasibility for each task
            infeasible_tasks = []
            
            for task_id in assigned_tasks:
                task = self.tasks[task_id]
                
                # Check if task is still feasible with current capabilities
                required_capabilities = np.array(task.capabilities_required)
                
                # Calculate capability match score
                if len(capabilities) > 0 and len(required_capabilities) > 0:
                    # Normalize vectors
                    capabilities_norm = capabilities / np.linalg.norm(capabilities)
                    required_norm = required_capabilities / np.linalg.norm(required_capabilities)
                    
                    # Compute match score (cosine similarity)
                    match_score = np.dot(capabilities_norm, required_norm)
                    
                    if match_score < self.capability_threshold:
                        infeasible_tasks.append(task_id)
                else:
                    # If capability vectors are empty, assume infeasible
                    infeasible_tasks.append(task_id)
            
            if infeasible_tasks:
                self.get_logger().info(
                    f'Found {len(infeasible_tasks)} infeasible tasks for Robot {robot_id}: {infeasible_tasks}')
                
                # Publish recovery status with infeasible tasks
                msg = RecoveryStatus()
                msg.robot_id = robot_id
                msg.recovery_type = "TASK_INFEASIBLE"
                msg.infeasible_tasks = infeasible_tasks
                msg.timestamp = int(time.time() * 1000)
                
                self.recovery_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PartialFailureRecoveryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()