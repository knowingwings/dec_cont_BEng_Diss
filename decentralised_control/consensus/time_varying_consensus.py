# decentralized_control/consensus/time_varying_consensus.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time
from threading import Lock

from decentralized_control.msg import RobotState, TaskAssignment, NetworkTopology

class TimeVaryingConsensusNode(Node):
    """
    Implements advanced time-weighted consensus protocol with support for
    asynchronous updates, time-varying communication delays, and changing topologies.
    """
    
    def __init__(self):
        super().__init__('time_varying_consensus_node')
        
        # Get robot ID from parameter
        self.declare_parameter('robot_id', 1)
        self.robot_id = self.get_parameter('robot_id').value
        
        # Initialize consensus parameters
        self.declare_parameter('gamma', 0.5)
        self.declare_parameter('lambda', 0.1)
        self.declare_parameter('max_delay', 0.5)  # Maximum acceptable delay in seconds
        self.declare_parameter('min_update_period', 0.05)  # Minimum time between updates (s)
        
        # Get parameters
        self.gamma = self.get_parameter('gamma').value
        self.lambda_val = self.get_parameter('lambda').value
        self.max_delay = self.get_parameter('max_delay').value
        self.min_update_period = self.get_parameter('min_update_period').value
        
        # Initialize internal state variables
        self.state_vector = None  # Will be initialized when first data is received
        self.state_vectors = {}  # Robot ID -> state vector
        self.last_update_time = {}  # Robot ID -> last update time
        self.last_state_update = 0  # Time of last state update
        self.network_topology = np.zeros((2, 2))  # Adjacency matrix for the network
        self.lock = Lock()
        
        # Publishers
        self.state_publisher = self.create_publisher(
            RobotState, f'/robot{self.robot_id}/state', 10)
        
        self.topology_publisher = self.create_publisher(
            NetworkTopology, '/network/topology', 10)
        
        # Subscribers
        self.state_subscribers = []
        for i in range(1, 3):  # For dual mobile manipulators
            if i != self.robot_id:
                self.state_subscribers.append(
                    self.create_subscription(
                        RobotState, f'/robot{i}/state', self.state_callback, 10))
        
        self.topology_subscriber = self.create_subscription(
            NetworkTopology, '/network/topology', self.topology_callback, 10)
        
        self.assignment_subscriber = self.create_subscription(
            TaskAssignment, '/auction/assignments', self.assignment_callback, 10)
        
        # Timer for consensus updates with asynchronous behavior
        self.next_update_time = time.time() + self.min_update_period * np.random.random()
        self.consensus_timer = self.create_timer(
            self.min_update_period / 2, self.asynchronous_update_callback)
        
        self.get_logger().info(
            f'Time-varying consensus node initialized for Robot {self.robot_id}')
    
    def state_callback(self, msg):
        """Store state information from other robots with timestamp."""
        with self.lock:
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
            
            # Store state vector with timestamp
            self.state_vectors[robot_id] = other_state
            self.last_update_time[robot_id] = time.time()
    
    def topology_callback(self, msg):
        """Update network topology information."""
        with self.lock:
            # Update adjacency matrix
            self.network_topology = np.array(msg.adjacency_matrix).reshape((msg.num_robots, msg.num_robots))
            
            # Update network properties
            # Calculate Laplacian matrix
            self.laplacian = np.diag(np.sum(self.network_topology, axis=1)) - self.network_topology
            
            # Calculate algebraic connectivity (second smallest eigenvalue of Laplacian)
            eigenvalues = np.sort(np.linalg.eigvals(self.laplacian))
            self.algebraic_connectivity = eigenvalues[1] if len(eigenvalues) > 1 else 0
            
            # Calculate theoretical convergence rate
            if self.algebraic_connectivity > 0:
                self.convergence_rate = -np.log(1 - self.gamma * self.algebraic_connectivity)
                
                self.get_logger().info(
                    f'Network topology updated: algebraic connectivity = {self.algebraic_connectivity:.4f}, '
                    f'convergence rate = {self.convergence_rate:.4f}')
    
    def assignment_callback(self, msg):
        """Process task assignment updates."""
        # This will be handled by the auction node
        pass
    
    def asynchronous_update_callback(self):
        """
        Perform asynchronous time-weighted consensus updates.
        Only updates when next_update_time is reached.
        """
        current_time = time.time()
        
        # Only update if it's time for this robot's update
        if current_time >= self.next_update_time:
            # Schedule next update with random offset to simulate asynchronous behavior
            random_offset = self.min_update_period * (0.5 + np.random.random())
            self.next_update_time = current_time + random_offset
            
            # Perform consensus update
            self.consensus_update_callback()
    
    def consensus_update_callback(self):
        """Perform a time-weighted consensus update with respect to current network topology."""
        with self.lock:
            if self.state_vector is None or not self.state_vectors:
                return  # Not enough information yet
            
            # Start with our current state
            x_consensus = self.state_vector.copy()
            
            # Update state based on information from other robots with time-weighting
            current_time = time.time()
            for robot_id, other_state in self.state_vectors.items():
                # Skip if link is not active in current topology
                if self.network_topology[self.robot_id-1, robot_id-1] == 0:
                    continue
                    
                # Calculate time-weighted factor with exponential decay
                time_diff = current_time - self.last_update_time.get(robot_id, 0)
                
                # Skip if delay exceeds maximum
                if time_diff > self.max_delay:
                    self.get_logger().warn(
                        f'Skipping update from Robot {robot_id}: delay {time_diff:.3f}s exceeds maximum {self.max_delay}s')
                    continue
                
                # Time-weighted factor with exponential decay
                weight = self.gamma * np.exp(-self.lambda_val * time_diff)
                
                # Update state
                x_consensus += weight * (other_state - self.state_vector)
            
            # Update our state vector
            self.state_vector = x_consensus
            self.last_state_update = current_time
            
            # Publish updated state
            self.publish_state()
    
    def publish_state(self):
        """Publish current state information."""
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
        
        # Add timestamp
        msg.timestamp = int(time.time() * 1000)  # milliseconds
        
        self.state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TimeVaryingConsensusNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()