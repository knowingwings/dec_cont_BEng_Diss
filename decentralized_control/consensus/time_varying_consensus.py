# decentralized_control/consensus/time_varying_consensus.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time
from threading import Lock

from dec_control.msg import RobotState, TaskAssignment, NetworkTopology

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
        
        # Add adaptive convergence parameters
        self.declare_parameter('adapt_rate', 0.1)  # Rate of adaptation
        self.declare_parameter('min_gamma', 0.1)   # Minimum gamma value
        self.declare_parameter('max_gamma', 0.9)   # Maximum gamma value
        
        # Get parameters
        self.gamma = self.get_parameter('gamma').value
        self.lambda_val = self.get_parameter('lambda').value
        self.max_delay = self.get_parameter('max_delay').value
        self.min_update_period = self.get_parameter('min_update_period').value
        self.adapt_rate = self.get_parameter('adapt_rate').value
        self.min_gamma = self.get_parameter('min_gamma').value
        self.max_gamma = self.get_parameter('max_gamma').value
        self.base_gamma = self.gamma
        
        # Initialize internal state variables
        self.state_vector = None  # Will be initialized when first data is received
        self.state_vectors = {}  # Robot ID -> state vector
        self.last_update_time = {}  # Robot ID -> last update time
        self.last_state_update = 0  # Time of last state update
        self.network_topology = np.zeros((2, 2))  # Adjacency matrix for the network
        self.lock = Lock()
        
        # Network quality metrics
        self.link_quality = np.ones((2, 2))      # From NetworkTopology
        self.link_latency = np.zeros((2, 2))     # From NetworkTopology
        self.link_reliability = np.ones((2, 2))  # From NetworkTopology
        self.last_topology_update = 0
        
        # Track convergence metrics
        self.prev_state = None
        self.convergence_error = float('inf')
        self.state_change_rate = 0.0
        self.last_log_time = time.time()
        
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
    
    def configure(self, delay_mean=0.0, delay_stddev=0.0, packet_loss=0.0):
        """
        Configure network conditions for testing adaptability.
        
        Parameters:
        -----------
        delay_mean : float
            Mean communication delay in seconds
        delay_stddev : float
            Standard deviation of communication delay
        packet_loss : float
            Probability of packet loss (0.0-1.0)
        """
        with self.lock:
            # Update parameters
            self.max_delay = max(0.5, delay_mean * 3)  # Adaptive maximum delay
            
            # Update convergence rate estimation based on network conditions
            # Higher delays and packet loss reduce effective algebraic connectivity
            effective_connectivity = self.algebraic_connectivity
            if hasattr(self, 'algebraic_connectivity'):
                degradation_factor = 1.0 / (1.0 + delay_mean * 5 + packet_loss * 10)
                effective_connectivity = self.algebraic_connectivity * degradation_factor
                
                if effective_connectivity > 0:
                    self.convergence_rate = -np.log(1 - self.gamma * effective_connectivity)
                    
                    self.get_logger().info(
                        f'Network conditions updated: delay={delay_mean}±{delay_stddev}s, '
                        f'packet_loss={packet_loss}, convergence_rate={self.convergence_rate:.4f}')

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
        """Update network topology and link quality information."""
        with self.lock:
            # Update adjacency matrix
            self.network_topology = np.array(msg.adjacency_matrix).reshape((msg.num_robots, msg.num_robots))
            
            # Update link metrics
            self.link_quality = np.array(msg.link_quality).reshape((msg.num_robots, msg.num_robots))
            self.link_latency = np.array(msg.link_latency).reshape((msg.num_robots, msg.num_robots))
            self.link_reliability = np.array(msg.link_reliability).reshape((msg.num_robots, msg.num_robots))
            
            # Update network properties
            self.laplacian = np.diag(np.sum(self.network_topology, axis=1)) - self.network_topology
            eigenvalues = np.sort(np.linalg.eigvals(self.laplacian))
            self.algebraic_connectivity = eigenvalues[1] if len(eigenvalues) > 1 else 0
            
            # Update convergence parameters based on network conditions
            if self.algebraic_connectivity > 0:
                # Calculate theoretical convergence rate
                self.convergence_rate = -np.log(1 - self.gamma * self.algebraic_connectivity)
                
                # Adjust gamma based on network conditions
                avg_quality = np.mean(self.link_quality[self.link_quality > 0])
                avg_reliability = np.mean(self.link_reliability[self.link_reliability > 0])
                network_health = avg_quality * avg_reliability
                
                # Scale gamma inversely with network health
                self.gamma = np.clip(
                    self.base_gamma * (1 + (1 - network_health)),
                    self.min_gamma,
                    self.max_gamma
                )
            
            self.last_topology_update = time.time()
            
            self.get_logger().info(
                f'Network metrics updated - Quality: {avg_quality:.3f}, '
                f'Reliability: {avg_reliability:.3f}, '
                f'Adjusted gamma: {self.gamma:.3f}')
    
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
        """Perform consensus update with network-aware weighting."""
        with self.lock:
            if self.state_vector is None or not self.state_vectors:
                return
            
            current_time = time.time()
            
            # Store previous state for convergence analysis
            if self.prev_state is not None:
                self.state_change_rate = np.linalg.norm(self.state_vector - self.prev_state)
            self.prev_state = self.state_vector.copy()
            
            # Start with our current state
            x_consensus = self.state_vector.copy()
            total_weight = 1.0
            error_sum = 0.0
            
            for robot_id, other_state in self.state_vectors.items():
                i, j = self.robot_id-1, robot_id-1
                
                # Skip if link is inactive
                if self.network_topology[i,j] == 0:
                    continue
                
                # Calculate time since last update
                time_diff = current_time - self.last_update_time.get(robot_id, 0)
                if time_diff > self.max_delay:
                    continue
                
                # Calculate network quality factors
                time_factor = np.exp(-self.lambda_val * time_diff)
                quality_factor = self.link_quality[i,j]
                reliability_factor = self.link_reliability[i,j]
                latency_factor = 1.0 / (1.0 + self.link_latency[i,j])
                
                # Combine factors for adaptive weight
                network_health = quality_factor * reliability_factor * latency_factor
                
                # Calculate consensus weight with network awareness
                weight = self.gamma * time_factor * network_health
                
                # Calculate error and update consensus state
                error = np.linalg.norm(other_state - self.state_vector)
                error_sum += error
                
                x_consensus += weight * (other_state - self.state_vector)
                total_weight += weight
            
            # Normalize by total weight
            x_consensus /= total_weight
            
            # Update convergence metrics
            self.convergence_error = error_sum / len(self.state_vectors) if self.state_vectors else float('inf')
            
            # Adapt gamma based on network conditions and convergence
            self._adapt_consensus_parameter(self.convergence_error, self.state_change_rate)
            
            # Update state vector and tracking variables
            self.state_vector = x_consensus
            self.last_state_update = current_time
            
            # Log metrics periodically
            if current_time - self.last_log_time > 5.0:  # Log every 5 seconds
                self.get_logger().info(
                    f'Convergence metrics - Error: {self.convergence_error:.3f}, '
                    f'Change rate: {self.state_change_rate:.3f}, '
                    f'Gamma: {self.gamma:.3f}'
                )
                self.last_log_time = current_time
            
            # Publish updated state
            self.publish_state()
    
    def _adapt_consensus_parameter(self, error, change_rate):
        """Adapt consensus parameter based on network conditions and convergence."""
        with self.lock:
            # Calculate network health metrics
            active_links = self.network_topology > 0
            if not np.any(active_links):
                return
            
            avg_quality = np.mean(self.link_quality[active_links])
            avg_reliability = np.mean(self.link_reliability[active_links])
            avg_latency = np.mean(self.link_latency[active_links])
            
            # Calculate desired gamma adjustment
            network_health = avg_quality * avg_reliability / (1 + avg_latency)
            error_factor = np.clip(error / self.convergence_error if self.convergence_error > 0 else 1.0, 0.1, 10.0)
            
            # More conservative when error is high, more aggressive when network conditions are good
            target_gamma = self.base_gamma * network_health / error_factor
            
            # Smooth adaptation
            self.gamma = np.clip(
                self.gamma + self.adapt_rate * (target_gamma - self.gamma),
                self.min_gamma,
                self.max_gamma
            )
    
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