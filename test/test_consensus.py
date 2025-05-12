#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
import time
import numpy as np
from threading import Thread

from decentralized_control.consensus.consensus_node import ConsensusNode
from decentralized_control.msg import RobotState

class TestConsensus(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.consensus_node1 = ConsensusNode()
        cls.consensus_node2 = ConsensusNode()
        
        # Override parameters
        cls.consensus_node1.robot_id = 1
        cls.consensus_node2.robot_id = 2
        
        # Start spinning in separate threads
        cls.node1_thread = Thread(target=lambda: rclpy.spin(cls.consensus_node1))
        cls.node2_thread = Thread(target=lambda: rclpy.spin(cls.consensus_node2))
        
        cls.node1_thread.daemon = True
        cls.node2_thread.daemon = True
        
        cls.node1_thread.start()
        cls.node2_thread.start()
        
        # Wait for nodes to initialize
        time.sleep(1)
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def test_state_initialization(self):
        """Test that consensus nodes initialize correctly."""
        self.assertEqual(self.consensus_node1.robot_id, 1)
        self.assertEqual(self.consensus_node2.robot_id, 2)
        
        # Initial state vector should be None until first data is received
        self.assertIsNone(self.consensus_node1.state_vector)
        self.assertIsNone(self.consensus_node2.state_vector)
    
    def test_state_consensus(self):
        """Test that state information converges between robots."""
        # Create test state message
        msg1 = RobotState()
        msg1.id = 1
        msg1.position = [1.0, 2.0, 3.0]
        msg1.orientation = [0.1, 0.2, 0.3]
        msg1.capabilities = [1.0, 0.5, 0.8, 0.3, 0.9]
        msg1.workload = 5.0
        msg1.failed = False
        
        msg2 = RobotState()
        msg2.id = 2
        msg2.position = [4.0, 5.0, 6.0]
        msg2.orientation = [0.4, 0.5, 0.6]
        msg2.capabilities = [0.9, 0.7, 0.2, 0.6, 0.4]
        msg2.workload = 3.0
        msg2.failed = False
        
        # Manually trigger state callbacks
        self.consensus_node1.state_callback(msg2)
        self.consensus_node2.state_callback(msg1)
        
        # Wait for consensus updates
        time.sleep(0.5)
        
        # Both nodes should now have state vectors
        self.assertIsNotNone(self.consensus_node1.state_vector)
        self.assertIsNotNone(self.consensus_node2.state_vector)
    
    def test_time_weighted_consensus(self):
        """Test that time weighting affects consensus."""
        # Set different consensus parameters
        self.consensus_node1.gamma = 0.3
        self.consensus_node2.gamma = 0.7
        
        # Create divergent state messages
        msg1 = RobotState()
        msg1.id = 1
        msg1.position = [0.0, 0.0, 0.0]  # Starting point
        msg1.orientation = [0.0, 0.0, 0.0]
        msg1.capabilities = [1.0, 1.0, 1.0, 1.0, 1.0]
        msg1.workload = 0.0
        msg1.failed = False
        
        msg2 = RobotState()
        msg2.id = 2
        msg2.position = [10.0, 10.0, 10.0]  # Very different position
        msg2.orientation = [1.0, 1.0, 1.0]
        msg2.capabilities = [0.0, 0.0, 0.0, 0.0, 0.0]  # Very different capabilities
        msg2.workload = 10.0
        msg2.failed = False
        
        # Manually trigger state callbacks
        self.consensus_node1.state_callback(msg2)
        self.consensus_node2.state_callback(msg1)
        
        # Force consensus updates
        self.consensus_node1.consensus_update_callback()
        self.consensus_node2.consensus_update_callback()
        
        # Node 1 with lower gamma should be less influenced by node 2
        # Node 2 with higher gamma should be more influenced by node 1
        
        # Extract position from state vector
        pos1 = self.consensus_node1.state_vector[0:3]
        pos2 = self.consensus_node2.state_vector[0:3]
        
        # Node 1's position should be closer to 0 than to 10
        self.assertLess(np.linalg.norm(pos1), 5.0)
        
        # Node 2's position should be closer to 0 than to 10
        self.assertLess(np.linalg.norm(pos2), 5.0)
        
        # Node 2 should be more influenced by node 1 than vice versa
        self.assertLess(np.linalg.norm(pos1), np.linalg.norm(pos2))
    
    def test_consensus_convergence(self):
        """Test that consensus converges over time."""
        # Reset consensus parameters
        self.consensus_node1.gamma = 0.5
        self.consensus_node2.gamma = 0.5
        
        # Create different state messages
        msg1 = RobotState()
        msg1.id = 1
        msg1.position = [0.0, 0.0, 0.0]
        msg1.orientation = [0.0, 0.0, 0.0]
        msg1.capabilities = [1.0, 1.0, 1.0, 1.0, 1.0]
        msg1.workload = 0.0
        msg1.failed = False
        
        msg2 = RobotState()
        msg2.id = 2
        msg2.position = [10.0, 10.0, 10.0]
        msg2.orientation = [1.0, 1.0, 1.0]
        msg2.capabilities = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg2.workload = 10.0
        msg2.failed = False
        
        # Manually trigger state callbacks
        self.consensus_node1.state_callback(msg2)
        self.consensus_node2.state_callback(msg1)
        
        # Initial positions before consensus updates
        self.consensus_node1.consensus_update_callback()
        self.consensus_node2.consensus_update_callback()
        pos1_initial = np.array(self.consensus_node1.state_vector[0:3])
        pos2_initial = np.array(self.consensus_node2.state_vector[0:3])
        
        # Run multiple consensus updates
        for _ in range(10):
            self.consensus_node1.consensus_update_callback()
            self.consensus_node2.consensus_update_callback()
            time.sleep(0.1)
        
        # Final positions after consensus updates
        pos1_final = np.array(self.consensus_node1.state_vector[0:3])
        pos2_final = np.array(self.consensus_node2.state_vector[0:3])
        
        # Verify convergence
        # Positions should be closer to each other after updates
        initial_distance = np.linalg.norm(pos1_initial - pos2_initial)
        final_distance = np.linalg.norm(pos1_final - pos2_final)
        
        self.assertLess(final_distance, initial_distance)
        
        # Final positions should be converging to the same value
        self.assertLess(final_distance, 1.0)

if __name__ == '__main__':
    unittest.main()