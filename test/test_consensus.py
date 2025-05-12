#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
import time
import numpy as np
import yaml
from threading import Thread, Event
import os
from pathlib import Path

from decentralized_control.consensus.consensus_node import ConsensusNode
from decentralized_control.consensus.time_varying_consensus import TimeVaryingConsensusNode
from decentralized_control.communication_middleware import CommunicationMiddleware
from dec.msg import RobotState, NetworkTopology
from decentralized_control.metrics_collector import NetworkMetricsCollector

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
        final_distance = np.linalg.norm(pos1_final - pos2_initial)
        
        self.assertLess(final_distance, initial_distance)
        
        # Final positions should be converging to the same value
        self.assertLess(final_distance, 1.0)

class TestNetworkAwareConsensus(unittest.TestCase):
    """Test suite for network-aware consensus adaptation."""
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        
        # Load test configuration
        config_path = Path(__file__).parent.parent / 'config' / 'experiments.yaml'
        with open(config_path, 'r') as f:
            cls.config = yaml.safe_load(f)
        
        cls.test_done = Event()
        cls.spin_thread = Thread(target=cls.spin_nodes)
        cls.spin_thread.daemon = True
        cls.spin_thread.start()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        cls.spin_thread.join()
    
    @classmethod
    def spin_nodes(cls):
        while rclpy.ok():
            rclpy.spin_once(timeout_sec=0.1)
            if cls.test_done.is_set():
                break
    
    def setUp(self):
        """Set up test nodes for each test case."""
        # Create middleware with default configuration
        self.middleware = CommunicationMiddleware()
        self.middleware.configure()
        
        # Create metrics collector
        self.metrics = NetworkMetricsCollector()
        self.metrics.configure()
        
        # Create consensus nodes
        self.nodes = []
        for i in range(2):  # Test with dual robot system
            node = TimeVaryingConsensusNode()
            node.configure(robot_id=i+1)
            self.nodes.append(node)
        
        # Reset test completion flag
        self.__class__.test_done.clear()
    
    def tearDown(self):
        """Clean up test nodes."""
        self.__class__.test_done.set()
        for node in self.nodes:
            node.destroy_node()
        self.middleware.destroy_node()
        self.metrics.destroy_node()
    
    def test_adaptation_to_delay_increase(self):
        """Test adaptation when network delay increases."""
        # Set initial conditions
        initial_state = self.config['consensus_experiments']['initial_states']['divergent_position']
        for i, node in enumerate(self.nodes):
            robot_config = initial_state[f'robot{i+1}']
            node.state_vector = np.concatenate([
                robot_config['position'],
                robot_config['orientation'],
                robot_config['capabilities'],
                [robot_config['workload']]
            ])
        
        # Run with initial conditions
        time.sleep(5.0)  # Allow initial convergence
        
        # Record initial adaptation parameters
        initial_gammas = [node.gamma for node in self.nodes]
        
        # Increase network delay
        self.middleware.delay_mean = 0.5  # 500ms delay
        time.sleep(10.0)  # Allow adaptation
        
        # Check adaptation response
        adapted_gammas = [node.gamma for node in self.nodes]
        for initial, adapted in zip(initial_gammas, adapted_gammas):
            self.assertLess(adapted, initial, 
                          "Adaptation parameter should decrease with higher delay")
    
    def test_adaptation_to_packet_loss(self):
        """Test adaptation when packet loss increases."""
        # Initialize nodes
        for i, node in enumerate(self.nodes):
            node.state_vector = np.zeros(11)  # [pos, orient, cap, workload]
        
        # Run with ideal conditions
        time.sleep(5.0)
        
        # Record initial state
        initial_gammas = [node.gamma for node in self.nodes]
        initial_errors = self.metrics.get_consensus_errors()
        
        # Introduce packet loss
        self.middleware.packet_loss_prob = 0.3
        time.sleep(10.0)
        
        # Check adaptation
        adapted_gammas = [node.gamma for node in self.nodes]
        final_errors = self.metrics.get_consensus_errors()
        
        # Verify adaptation improved performance
        error_reduction = np.mean(final_errors) / np.mean(initial_errors)
        self.assertLess(error_reduction, 1.0,
                       "Adaptation should reduce consensus error under packet loss")
    
    def test_adaptation_to_varying_conditions(self):
        """Test adaptation under time-varying network conditions."""
        # Configure time-varying delay pattern
        self.middleware.configure(
            delay_pattern='sinusoidal',
            base_delay_ms=100,
            amplitude_ms=50,
            period_s=10.0
        )
        
        # Run experiment
        test_duration = 30.0  # 3 periods
        start_time = time.time()
        
        gammas = []
        delays = []
        
        while time.time() - start_time < test_duration:
            # Record current state
            current_gammas = [node.gamma for node in self.nodes]
            current_delay = self.middleware.calculate_current_delay()
            
            gammas.append(np.mean(current_gammas))
            delays.append(current_delay)
            
            time.sleep(0.1)
        
        # Calculate correlation between delay and adaptation
        correlation = np.corrcoef(delays, gammas)[0,1]
        
        # Verify negative correlation (gamma should decrease with delay)
        self.assertLess(correlation, -0.5,
                       "Adaptation parameter should track network delay changes")
    
    def test_message_rate_adaptation(self):
        """Test adaptation of message rates to network conditions."""
        # Get initial message rates
        time.sleep(5.0)  # Allow initial convergence
        initial_rates = self.metrics.get_message_rates()
        
        # Introduce poor network conditions
        self.middleware.configure(
            delay_mean_ms=300,
            delay_stddev_ms=50,
            packet_loss_prob=0.2
        )
        
        time.sleep(10.0)  # Allow adaptation
        
        # Get adapted message rates
        adapted_rates = self.metrics.get_message_rates()
        
        # Verify rate reduction
        self.assertLess(np.mean(adapted_rates), np.mean(initial_rates),
                       "Message rates should reduce under poor network conditions")
    
    def test_convergence_under_burst_traffic(self):
        """Test convergence maintenance under burst network traffic."""
        # Configure burst traffic pattern
        self.middleware.configure(
            delay_pattern='burst',
            base_delay_ms=50,
            burst_interval_s=5.0,
            burst_duration_s=2.0,
            burst_multiplier=5.0
        )
        
        # Set divergent initial states
        initial_state = self.config['consensus_experiments']['initial_states']['divergent_position']
        for i, node in enumerate(self.nodes):
            robot_config = initial_state[f'robot{i+1}']
            node.state_vector = np.concatenate([
                robot_config['position'],
                robot_config['orientation'],
                robot_config['capabilities'],
                [robot_config['workload']]
            ])
        
        # Run experiment through multiple burst cycles
        test_duration = 20.0  # 4 burst cycles
        start_time = time.time()
        max_error_increase = 0.0
        last_error = float('inf')
        
        while time.time() - start_time < test_duration:
            current_error = np.mean(self.metrics.get_consensus_errors())
            
            # Track maximum error increase during bursts
            if current_error > last_error:
                error_increase = (current_error - last_error) / last_error
                max_error_increase = max(max_error_increase, error_increase)
            
            last_error = current_error
            time.sleep(0.1)
        
        # Verify error increases during bursts are bounded
        self.assertLess(max_error_increase, 2.0,
                       "Error increases during traffic bursts should be bounded")
    
    def test_stability_bounds(self):
        """Test that adaptation maintains stability bounds."""
        # Initialize with worst-case network conditions
        self.middleware.configure(
            delay_mean_ms=500,
            delay_stddev_ms=100,
            packet_loss_prob=0.3
        )
        
        # Set divergent initial states
        initial_state = self.config['consensus_experiments']['initial_states']['divergent_position']
        for i, node in enumerate(self.nodes):
            robot_config = initial_state[f'robot{i+1}']
            node.state_vector = np.concatenate([
                robot_config['position'],
                robot_config['orientation'],
                robot_config['capabilities'],
                [robot_config['workload']]
            ])
        
        # Run experiment
        test_duration = 30.0
        start_time = time.time()
        max_gamma = 0.0
        min_gamma = float('inf')
        
        while time.time() - start_time < test_duration:
            # Track gamma bounds
            for node in self.nodes:
                max_gamma = max(max_gamma, node.gamma)
                min_gamma = min(min_gamma, node.gamma)
            
            time.sleep(0.1)
        
        # Verify gamma stays within stability bounds
        self.assertGreater(min_gamma, 0.1,
                          "Adaptation parameter should maintain minimum bound")
        self.assertLess(max_gamma, 0.9,
                       "Adaptation parameter should maintain maximum bound")
        
        # Verify final error is bounded
        final_errors = self.metrics.get_consensus_errors()
        self.assertLess(np.mean(final_errors), 1.0,
                       "Consensus error should remain bounded under worst-case conditions")

if __name__ == '__main__':
    unittest.main()