#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time

from dec_control.msg import RobotState, TaskAssignment

class ConsensusNode(Node):
    """
    Implements the time-weighted consensus protocol for maintaining
    consistent information between robots.
    """
    
    def __init__(self):
        super().__init__('consensus_node')
        
        # Get robot ID from parameter
        self.declare_parameter('robot_id', 1)
        self.robot_id = self.get_parameter('robot_id').value
        
        # Initialize consensus parameters
        self.declare_parameter('gamma', 0.5)
        self.declare_parameter('lambda', 0.1)
        
        # Get parameters
        self.gamma = self.get_parameter('gamma').value
        self.lambda_val = self.get_parameter('lambda').value
        
        # Initialize internal state variables
        self.state_vector = None  # Will be initialized when first data is received
        self.state_vectors = {}  # Robot ID -> state vector
        self.last_update_time = {}  # Robot ID -> last update time
        
        # Publishers
        self.state_publisher = self.create_publisher(
            RobotState, f'/robot{self.robot_id}/state', 10)
        
        self.assignment_publisher = self.create_publisher(
            TaskAssignment, '/auction/assignments', 10)
        
        # Subscribers
        self.state_subscribers = []
        for i in range(1, 3):  # For dual mobile manipulators
            if i != self.robot_id:
                self.state_subscribers.append(
                    self.create_subscription(
                        RobotState, f'/robot{i}/state', self.state_callback, 10))
        
        self.assignment_subscriber = self.create_subscription(
            TaskAssignment, '/auction/assignments', self.assignment_callback, 10)
        
        # Timer for consensus updates
        self.consensus_timer = self.create_timer(
            0.1, self.consensus_update_callback)  # 10Hz updates
        
        self.get_logger().info(
            f'Consensus node initialized for Robot {self.robot_id}')
    
    def state_callback(self, msg):
        """Store state information from other robots"""
        robot_id = msg.id
        
        # Create state vector if not already created
        if self.state_vector is None:
            # Initialize with zeros
            self.state_vector = np.zeros(3 + 3 + len(msg.capabilities) + 2)
        
        # Update state vector from other robot
        other_state = np.concatenate([
            msg.position,
            msg.orientation,
            msg.capabilities,
            [msg.workload, 1.0 if msg.failed else 0.0]
        ])
        
        # Store state vector
        self.state_vectors[robot_id] = other_state
        
        # Update last update time
        self.last_update_time[robot_id] = time.time()
    
    def assignment_callback(self, msg):
        """Process task assignment updates"""
        # This will be handled by the auction node
        pass
    
    def consensus_update_callback(self):
        """Perform a time-weighted consensus update"""
        if self.state_vector is None or not self.state_vectors:
            return  # Not enough information yet
        
        # Start with our current state
        x_consensus = self.state_vector.copy()
        
        # Update state based on information from other robots
        current_time = time.time()
        for robot_id, other_state in self.state_vectors.items():
            # Calculate time-weighted factor
            time_diff = current_time - self.last_update_time.get(robot_id, 0)
            weight = self.gamma * np.exp(-self.lambda_val * time_diff)
            
            # Update state
            x_consensus += weight * (other_state - self.state_vector)
        
        # Update our state vector
        self.state_vector = x_consensus
        
        # Publish updated state
        self.publish_state()
    
    def publish_state(self):
        """Publish current state information"""
        if self.state_vector is None:
            return
        
        msg = RobotState()
        msg.id = self.robot_id
        
        # Extract components from state vector
        msg.position = self.state_vector[0:3].tolist()
        msg.orientation = self.state_vector[3:6].tolist()
        
        # Extract capabilities based on length (can vary)
        capability_length = len(self.state_vector) - 8  # Subtract position, orientation, workload, failed
        msg.capabilities = self.state_vector[6:6+capability_length].tolist()
        
        # Extract workload and failed status
        msg.workload = float(self.state_vector[-2])
        msg.failed = bool(self.state_vector[-1] > 0.5)
        
        self.state_publisher.publish(msg)