#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
import time
import numpy as np
from threading import Thread

from decentralized_control.auction.bid_calculator import BidCalculator
from decentralized_control.auction.auction_node import AuctionNode
from dec_control.msg import Task, TaskList, Bid, TaskAssignment, RobotState

class TestTaskPublisher(Node):
    """Test node for publishing tasks."""
    
    def __init__(self):
        super().__init__('test_task_publisher')
        self.publisher = self.create_publisher(TaskList, '/tasks', 10)
    
    def publish_tasks(self, num_tasks=5):
        """Publish a set of test tasks."""
        msg = TaskList()
        
        for i in range(1, num_tasks + 1):
            task = Task()
            task.id = i
            task.position = [float(i), 0.0, 0.0]
            task.capabilities_required = [1.0, 1.0, 1.0, 1.0, 1.0]
            task.execution_time = 5.0
            task.prerequisites = []
            task.requires_collaboration = False
            
            msg.tasks.append(task)
        
        self.publisher.publish(msg)

class AssignmentCollector(Node):
    """Test node for collecting task assignments."""
    
    def __init__(self):
        super().__init__('assignment_collector')
        self.subscription = self.create_subscription(
            TaskAssignment, '/auction/assignments', self.assignment_callback, 10)
        self.assignments = {}
        self.prices = {}
        self.update_count = 0
    
    def assignment_callback(self, msg):
        for i in range(len(msg.task_ids)):
            task_id = msg.task_ids[i]
            robot_id = msg.robot_ids[i]
            price = msg.prices[i]
            
            self.assignments[task_id] = robot_id
            self.prices[task_id] = price
        
        self.update_count += 1

class TestAuction(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.auction_node1 = AuctionNode()
        cls.auction_node2 = AuctionNode()
        
        # Override parameters
        cls.auction_node1.robot_id = 1
        cls.auction_node2.robot_id = 2
        
        # Position the robots at different locations
        cls.auction_node1.position = np.array([0.0, 0.0, 0.0])
        cls.auction_node2.position = np.array([5.0, 0.0, 0.0])
        
        cls.task_publisher = TestTaskPublisher()
        cls.assignment_collector = AssignmentCollector()
        
        # Start spinning in separate threads
        cls.node1_thread = Thread(target=lambda: rclpy.spin(cls.auction_node1))
        cls.node2_thread = Thread(target=lambda: rclpy.spin(cls.auction_node2))
        cls.task_pub_thread = Thread(target=lambda: rclpy.spin(cls.task_publisher))
        cls.assign_thread = Thread(target=lambda: rclpy.spin(cls.assignment_collector))
        
        cls.node1_thread.daemon = True
        cls.node2_thread.daemon = True
        cls.task_pub_thread.daemon = True
        cls.assign_thread.daemon = True
        
        cls.node1_thread.start()
        cls.node2_thread.start()
        cls.task_pub_thread.start()
        cls.assign_thread.start()
        
        # Wait for nodes to initialize
        time.sleep(1)
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.assignment_collector.assignments = {}
        self.assignment_collector.prices = {}
        self.assignment_collector.update_count = 0
    
    def test_bid_calculator(self):
        """Test the bid calculation function."""
        calculator = BidCalculator([0.8, 0.3, 1.0, 1.2, 0.2], [2.0, 1.5])
        
        # Test basic bid calculation
        bid = calculator.calculate_bid(
            robot_id=1,
            task_id=1,
            robot_position=np.array([0.0, 0.0, 0.0]),
            task_position=np.array([1.0, 0.0, 0.0]),
            robot_capabilities=np.array([1.0, 1.0, 1.0, 1.0, 1.0]),
            task_capabilities=np.array([1.0, 1.0, 1.0, 1.0, 1.0]),
            robot_workload=0.0,
            workload_ratio=1.0,
            workload_imbalance=0.0
        )
        
        # Bid should be positive
        self.assertGreater(bid, 0.0)
        
        # Test bid with different distances
        bid_close = calculator.calculate_bid(
            robot_id=1,
            task_id=1,
            robot_position=np.array([0.0, 0.0, 0.0]),
            task_position=np.array([0.1, 0.0, 0.0]),
            robot_capabilities=np.array([1.0, 1.0, 1.0, 1.0, 1.0]),
            task_capabilities=np.array([1.0, 1.0, 1.0, 1.0, 1.0]),
            robot_workload=0.0,
            workload_ratio=1.0,
            workload_imbalance=0.0
        )
        
        bid_far = calculator.calculate_bid(
            robot_id=1,
            task_id=1,
            robot_position=np.array([0.0, 0.0, 0.0]),
            task_position=np.array([10.0, 0.0, 0.0]),
            robot_capabilities=np.array([1.0, 1.0, 1.0, 1.0, 1.0]),
            task_capabilities=np.array([1.0, 1.0, 1.0, 1.0, 1.0]),
            robot_workload=0.0,
            workload_ratio=1.0,
            workload_imbalance=0.0
        )
        
        # Closer task should have higher bid
        self.assertGreater(bid_close, bid_far)
        
        # Test bid with workload
        bid_no_load = calculator.calculate_bid(
            robot_id=1,
            task_id=1,
            robot_position=np.array([0.0, 0.0, 0.0]),
            task_position=np.array([1.0, 0.0, 0.0]),
            robot_capabilities=np.array([1.0, 1.0, 1.0, 1.0, 1.0]),
            task_capabilities=np.array([1.0, 1.0, 1.0, 1.0, 1.0]),
            robot_workload=0.0,
            workload_ratio=0.5,
            workload_imbalance=0.5
        )
        
        bid_high_load = calculator.calculate_bid(
            robot_id=1,
            task_id=1,
            robot_position=np.array([0.0, 0.0, 0.0]),
            task_position=np.array([1.0, 0.0, 0.0]),
            robot_capabilities=np.array([1.0, 1.0, 1.0, 1.0, 1.0]),
            task_capabilities=np.array([1.0, 1.0, 1.0, 1.0, 1.0]),
            robot_workload=10.0,
            workload_ratio=1.5,
            workload_imbalance=0.5
        )
        
        # Lower workload should have higher bid
        self.assertGreater(bid_no_load, bid_high_load)
    
    def test_task_allocation(self):
        """Test that tasks are allocated between robots."""
        # Publish tasks
        self.task_publisher.publish_tasks(num_tasks=5)
        
        # Wait for auction to converge
        max_wait = 5  # seconds
        start_time = time.time()
        while time.time() - start_time < max_wait:
            if len(self.assignment_collector.assignments) == 5:
                break
            time.sleep(0.1)
        
        # Verify all tasks were assigned
        self.assertEqual(len(self.assignment_collector.assignments), 5)
        
        # Verify each task has a valid robot assigned
        for task_id, robot_id in self.assignment_collector.assignments.items():
            self.assertIn(robot_id, [1, 2])
        
        # Verify prices are positive
        for task_id, price in self.assignment_collector.prices.items():
            self.assertGreaterEqual(price, 0.0)
    
    def test_distance_based_allocation(self):
        """Test that tasks are allocated based on distance."""
        # Override robot positions to make allocation predictable
        self.auction_node1.position = np.array([0.0, 0.0, 0.0])
        self.auction_node2.position = np.array([10.0, 0.0, 0.0])
        
        # Publish tasks with positions
        msg = TaskList()
        
        # Two tasks clearly closer to robot 1
        for i in range(1, 3):
            task = Task()
            task.id = i
            task.position = [float(i), 0.0, 0.0]  # Close to robot 1
            task.capabilities_required = [1.0, 1.0, 1.0, 1.0, 1.0]
            task.execution_time = 5.0
            task.prerequisites = []
            task.requires_collaboration = False
            
            msg.tasks.append(task)
        
        # Two tasks clearly closer to robot 2
        for i in range(3, 5):
            task = Task()
            task.id = i
            task.position = [float(i + 6), 0.0, 0.0]  # Close to robot 2
            task.capabilities_required = [1.0, 1.0, 1.0, 1.0, 1.0]
            task.execution_time = 5.0
            task.prerequisites = []
            task.requires_collaboration = False
            
            msg.tasks.append(task)
        
        self.task_publisher.publisher.publish(msg)
        
        # Wait for auction to converge
        max_wait = 5  # seconds
        start_time = time.time()
        while time.time() - start_time < max_wait:
            if len(self.assignment_collector.assignments) == 4:
                break
            time.sleep(0.1)
        
        # Verify assignment based on distance
        # Tasks 1-2 should be assigned to robot 1
        # Tasks 3-4 should be assigned to robot 2
        for task_id in [1, 2]:
            self.assertEqual(self.assignment_collector.assignments.get(task_id), 1,
                            f"Task {task_id} should be assigned to robot 1")
        
        for task_id in [3, 4]:
            self.assertEqual(self.assignment_collector.assignments.get(task_id), 2,
                            f"Task {task_id} should be assigned to robot 2")
    
    def test_epsilon_impact(self):
        """Test impact of epsilon on auction convergence."""
        # Test with small epsilon
        self.auction_node1.epsilon = 0.01
        self.auction_node2.epsilon = 0.01
        
        # Reset assignments
        self.assignment_collector.assignments = {}
        self.assignment_collector.prices = {}
        self.assignment_collector.update_count = 0
        
        # Publish tasks
        self.task_publisher.publish_tasks(num_tasks=3)
        
        # Wait for auction to converge
        time.sleep(2)
        updates_small_epsilon = self.assignment_collector.update_count
        
        # Test with larger epsilon
        self.auction_node1.epsilon = 0.5
        self.auction_node2.epsilon = 0.5
        
        # Reset assignments
        self.assignment_collector.assignments = {}
        self.assignment_collector.prices = {}
        self.assignment_collector.update_count = 0
        
        # Publish tasks
        self.task_publisher.publish_tasks(num_tasks=3)
        
        # Wait for auction to converge
        time.sleep(2)
        updates_large_epsilon = self.assignment_collector.update_count
        
        # Larger epsilon should result in fewer updates (faster convergence)
        self.assertLess(updates_large_epsilon, updates_small_epsilon)

if __name__ == '__main__':
    unittest.main()