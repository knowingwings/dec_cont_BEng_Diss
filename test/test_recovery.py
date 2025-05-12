#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
import time
from threading import Thread

from dec_control.recovery.recovery_node import RecoveryNode
from dec_control.msg import Heartbeat, RobotState, TaskAssignment

class TestRecovery(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.recovery_node1 = RecoveryNode()
        cls.recovery_node2 = RecoveryNode()
        
        # Override parameters
        cls.recovery_node1.robot_id = 1
        cls.recovery_node2.robot_id = 2
        
        # Reduce heartbeat timeout for faster testing
        cls.recovery_node1.heartbeat_timeout = 0.5
        cls.recovery_node2.heartbeat_timeout = 0.5
        
        # Create publisher nodes
        cls.heartbeat_pub1 = Node('heartbeat_publisher1')
        cls.heartbeat_pub2 = Node('heartbeat_publisher2')
        cls.state_pub1 = Node('state_publisher1')
        cls.state_pub2 = Node('state_publisher2')
        cls.task_assign_pub = Node('task_assign_publisher')
        
        # Create publishers
        cls.heartbeat_publisher1 = cls.heartbeat_pub1.create_publisher(
            Heartbeat, '/robot1/heartbeat', 10)
        cls.heartbeat_publisher2 = cls.heartbeat_pub2.create_publisher(
            Heartbeat, '/robot2/heartbeat', 10)
        cls.state_publisher1 = cls.state_pub1.create_publisher(
            RobotState, '/robot1/state', 10)
        cls.state_publisher2 = cls.state_pub2.create_publisher(
            RobotState, '/robot2/state', 10)
        cls.task_assign_publisher = cls.task_assign_pub.create_publisher(
            TaskAssignment, '/auction/assignments', 10)
        
        # Start spinning in separate threads
        cls.node1_thread = Thread(target=lambda: rclpy.spin(cls.recovery_node1))
        cls.node2_thread = Thread(target=lambda: rclpy.spin(cls.recovery_node2))
        cls.pub1_thread = Thread(target=lambda: rclpy.spin(cls.heartbeat_pub1))
        cls.pub2_thread = Thread(target=lambda: rclpy.spin(cls.heartbeat_pub2))
        cls.state1_thread = Thread(target=lambda: rclpy.spin(cls.state_pub1))
        cls.state2_thread = Thread(target=lambda: rclpy.spin(cls.state_pub2))
        cls.task_thread = Thread(target=lambda: rclpy.spin(cls.task_assign_pub))
        
        cls.node1_thread.daemon = True
        cls.node2_thread.daemon = True
        cls.pub1_thread.daemon = True
        cls.pub2_thread.daemon = True
        cls.state1_thread.daemon = True
        cls.state2_thread.daemon = True
        cls.task_thread.daemon = True
        
        cls.node1_thread.start()
        cls.node2_thread.start()
        cls.pub1_thread.start()
        cls.pub2_thread.start()
        cls.state1_thread.start()
        cls.state2_thread.start()
        cls.task_thread.start()
        
        # Wait for nodes to initialize
        time.sleep(1)
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def test_heartbeat_detection(self):
        """Test that heartbeats are detected correctly."""
        # Send heartbeat from robot 2
        heartbeat = Heartbeat()
        heartbeat.robot_id = 2
        heartbeat.timestamp = int(time.time() * 1000)
        heartbeat.status = 0
        
        self.heartbeat_publisher2.publish(heartbeat)
        
        # Wait for heartbeat to be processed
        time.sleep(0.1)
        
        # Verify robot 2 is not marked as failed
        self.assertEqual(self.recovery_node1.robot_statuses.get(2, 0), 0)
        
        # Wait for heartbeat timeout
        time.sleep(0.6)
        
        # Verify robot 2 is marked as suspected (state 1)
        self.assertEqual(self.recovery_node1.robot_statuses.get(2, 0), 1)
        
        # Wait for another heartbeat timeout to mark as failed
        time.sleep(0.6)
        
        # Verify robot 2 is marked as failed (state 2)
        self.assertEqual(self.recovery_node1.robot_statuses.get(2, 0), 2)
    
    def test_heartbeat_recovery(self):
        """Test that heartbeat recovery works correctly."""
        # Reset robot status
        self.recovery_node1.robot_statuses[2] = 0
        
        # Wait for heartbeat timeout to mark as suspected
        time.sleep(0.6)
        
        # Verify robot 2 is marked as suspected
        self.assertEqual(self.recovery_node1.robot_statuses.get(2, 0), 1)
        
        # Send heartbeat from robot 2
        heartbeat = Heartbeat()
        heartbeat.robot_id = 2
        heartbeat.timestamp = int(time.time() * 1000)
        heartbeat.status = 0
        
        self.heartbeat_publisher2.publish(heartbeat)
        
        # Wait for heartbeat to be processed
        time.sleep(0.1)
        
        # Verify robot 2 is no longer suspected
        self.assertEqual(self.recovery_node1.robot_statuses.get(2, 0), 0)
    
    def test_explicit_failure_detection(self):
        """Test that explicit failure reporting is detected."""
        # Send state message from robot 2 with failed=True
        state = RobotState()
        state.id = 2
        state.position = [0.0, 0.0, 0.0]
        state.orientation = [0.0, 0.0, 0.0]
        state.capabilities = [1.0, 1.0, 1.0, 1.0, 1.0]
        state.workload = 0.0
        state.failed = True
        
        self.state_publisher2.publish(state)
        
        # Wait for state to be processed
        time.sleep(0.1)
        
        # Verify robot 2 is marked as failed
        self.assertEqual(self.recovery_node1.robot_statuses.get(2, 0), 2)
        
        # Verify robot 2 is in the failed robots set
        self.assertIn(2, self.recovery_node1.failed_robots)
        
        # Verify recovery mode is activated
        self.assertTrue(self.recovery_node1.recovery_mode)
    
    def test_recovery_completion(self):
        """Test that recovery completion is detected."""
        # Reset recovery mode
        self.recovery_node1.recovery_mode = True
        self.recovery_node1.failed_robots = {2}
        
        # Create task assignment with tasks assigned to robot 2 (failed)
        assignment = TaskAssignment()
        assignment.task_ids = [1, 2, 3]
        assignment.robot_ids = [2, 2, 1]  # Tasks 1 and 2 assigned to robot 2
        assignment.prices = [0.0, 0.0, 0.0]
        
        self.task_assign_publisher.publish(assignment)
        
        # Wait for assignment to be processed
        time.sleep(0.1)
        
        # Verify recovery is not complete (tasks still assigned to failed robot)
        self.assertTrue(self.recovery_node1.recovery_mode)
        
        # Update assignment with all tasks reassigned
        assignment.task_ids = [1, 2, 3]
        assignment.robot_ids = [1, 1, 1]  # All tasks reassigned to robot 1
        assignment.prices = [0.0, 0.0, 0.0]
        
        self.task_assign_publisher.publish(assignment)
        
        # Wait for assignment to be processed
        time.sleep(0.1)
        
        # Verify recovery is complete
        self.assertFalse(self.recovery_node1.recovery_mode)
        self.assertNotEqual(self.recovery_node1.recovery_complete_time, 0)
    
    def test_recovery_timeout(self):
        """Test recovery timing measurement."""
        # Reset recovery state
        self.recovery_node1.recovery_mode = False
        self.recovery_node1.recovery_start_time = 0
        self.recovery_node1.recovery_complete_time = 0
        
        # Trigger recovery
        self.recovery_node1.initiate_recovery(2)
        
        # Verify recovery started
        self.assertTrue(self.recovery_node1.recovery_mode)
        self.assertNotEqual(self.recovery_node1.recovery_start_time, 0)
        
        # Create task assignment with all tasks reassigned
        assignment = TaskAssignment()
        assignment.task_ids = [1, 2, 3]
        assignment.robot_ids = [1, 1, 1]  # All tasks reassigned to robot 1
        assignment.prices = [0.0, 0.0, 0.0]
        
        self.task_assign_publisher.publish(assignment)
        
        # Wait for assignment to be processed
        time.sleep(0.1)
        
        # Verify recovery completed and time was measured
        self.assertFalse(self.recovery_node1.recovery_mode)
        self.assertNotEqual(self.recovery_node1.recovery_complete_time, 0)
        
        # Verify recovery duration is reasonable
        recovery_duration = self.recovery_node1.recovery_complete_time - self.recovery_node1.recovery_start_time
        self.assertGreater(recovery_duration, 0)
        self.assertLess(recovery_duration, 1.0)  # Should complete quickly in test

if __name__ == '__main__':
    unittest.main()