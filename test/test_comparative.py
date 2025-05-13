#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
import time
import os
import json
from threading import Thread

from decentralized_control.comparative_analysis import ComparativeAnalysis, AllocationMethod
from dec_control.msg import Task, TaskList, TaskAssignment

class AssignmentSubscriber(Node):
    """Test node for subscribing to comparative assignments."""
    
    def __init__(self):
        super().__init__('assignment_subscriber')
        self.subscription = self.create_subscription(
            TaskAssignment, '/comparative/assignments', self.assignment_callback, 10)
        self.assignments = {}
        self.update_count = 0
    
    def assignment_callback(self, msg):
        for i in range(len(msg.task_ids)):
            task_id = msg.task_ids[i]
            robot_id = msg.robot_ids[i]
            
            self.assignments[task_id] = robot_id
        
        self.update_count += 1

class TestComparative(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        
        # Create temporary output directory
        cls.temp_dir = '/tmp/decentralized_control_test'
        os.makedirs(cls.temp_dir, exist_ok=True)
        
        # Create comparative analysis nodes for each method
        cls.centralized_node = ComparativeAnalysis()
        cls.greedy_node = ComparativeAnalysis()
        cls.market_based_node = ComparativeAnalysis()
        cls.sequential_node = ComparativeAnalysis()
        
        # Override parameters
        cls.centralized_node.output_dir = cls.temp_dir
        cls.centralized_node.allocation_method = AllocationMethod.CENTRALIZED
        cls.centralized_node.num_tasks = 10
        cls.centralized_node.num_robots = 2
        
        cls.greedy_node.output_dir = cls.temp_dir
        cls.greedy_node.allocation_method = AllocationMethod.GREEDY
        cls.greedy_node.num_tasks = 10
        cls.greedy_node.num_robots = 2
        
        cls.market_based_node.output_dir = cls.temp_dir
        cls.market_based_node.allocation_method = AllocationMethod.MARKET_BASED
        cls.market_based_node.num_tasks = 10
        cls.market_based_node.num_robots = 2
        
        cls.sequential_node.output_dir = cls.temp_dir
        cls.sequential_node.allocation_method = AllocationMethod.SEQUENTIAL
        cls.sequential_node.num_tasks = 10
        cls.sequential_node.num_robots = 2
        
        # Create task publisher node
        cls.task_pub = Node('task_publisher')
        cls.task_publisher = cls.task_pub.create_publisher(
            TaskList, '/tasks', 10)
        
        # Create assignment subscriber
        cls.assignment_subscriber = AssignmentSubscriber()
        
        # Start spinning in separate threads
        cls.centralized_thread = Thread(target=lambda: rclpy.spin(cls.centralized_node))
        cls.greedy_thread = Thread(target=lambda: rclpy.spin(cls.greedy_node))
        cls.market_thread = Thread(target=lambda: rclpy.spin(cls.market_based_node))
        cls.sequential_thread = Thread(target=lambda: rclpy.spin(cls.sequential_node))
        cls.task_thread = Thread(target=lambda: rclpy.spin(cls.task_pub))
        cls.subscriber_thread = Thread(target=lambda: rclpy.spin(cls.assignment_subscriber))
        
        cls.centralized_thread.daemon = True
        cls.greedy_thread.daemon = True
        cls.market_thread.daemon = True
        cls.sequential_thread.daemon = True
        cls.task_thread.daemon = True
        cls.subscriber_thread.daemon = True
        
        # Only start one node at a time for testing
        cls.task_thread.start()
        cls.subscriber_thread.start()
        
        # Wait for nodes to initialize
        time.sleep(1)
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def publish_test_tasks(self):
        """Publish test tasks with varying positions."""
        task_list = TaskList()
        
        for i in range(1, 11):
            task = Task()
            task.id = i
            task.position = [float(i), 0.0, 0.0]
            task.capabilities_required = [1.0, 1.0, 1.0, 1.0, 1.0]
            task.execution_time = float(i)  # Varying execution times
            task.prerequisites = []
            task.requires_collaboration = False
            
            task_list.tasks.append(task)
        
        self.task_publisher.publish(task_list)
        
        # Wait for tasks to be processed
        time.sleep(0.1)
    
    def test_centralized_allocation(self):
        """Test centralized allocation method."""
        # Start centralized node
        self.centralized_thread.start()
        time.sleep(0.5)  # Wait for node to initialize
        
        # Reset subscriber
        self.assignment_subscriber.assignments = {}
        self.assignment_subscriber.update_count = 0
        
        # Publish tasks
        self.publish_test_tasks()
        
        # Wait for allocation
        time.sleep(0.5)
        
        # Verify assignments were made
        self.assertGreater(self.assignment_subscriber.update_count, 0)
        self.assertEqual(len(self.assignment_subscriber.assignments), 10)
        
        # Verify all tasks were assigned to a valid robot
        for task_id, robot_id in self.assignment_subscriber.assignments.items():
            self.assertIn(robot_id, [1, 2])
        
        # Verify results file was created
        filename = os.path.join(self.temp_dir, 'comparative_CENTRALIZED_T10.json')
        self.assertTrue(os.path.exists(filename))
        
        # Load and check results
        with open(filename, 'r') as f:
            results = json.load(f)
        
        self.assertEqual(results['allocation_method'], 'CENTRALIZED')
        self.assertEqual(results['num_tasks'], 10)
        self.assertEqual(results['num_robots'], 2)
        self.assertGreater(results['makespan'], 0)
        
        # Check for load balancing - difference should be relatively small
        robot_makespans = results['robot_makespans']
        makespan_values = list(map(float, robot_makespans.values()))
        makespan_diff = abs(makespan_values[0] - makespan_values[1])
        
        # In a well-balanced allocation, the difference should be small
        # relative to the total workload
        total_workload = sum(makespan_values)
        self.assertLess(makespan_diff / total_workload, 0.3)
    
    def test_greedy_allocation(self):
        """Test greedy allocation method."""
        # Start greedy node
        self.greedy_thread.start()
        time.sleep(0.5)  # Wait for node to initialize
        
        # Reset subscriber
        self.assignment_subscriber.assignments = {}
        self.assignment_subscriber.update_count = 0
        
        # Publish tasks
        self.publish_test_tasks()
        
        # Wait for allocation
        time.sleep(0.5)
        
        # Verify assignments were made
        self.assertGreater(self.assignment_subscriber.update_count, 0)
        self.assertEqual(len(self.assignment_subscriber.assignments), 10)
        
        # Verify results file was created
        filename = os.path.join(self.temp_dir, 'comparative_GREEDY_T10.json')
        self.assertTrue(os.path.exists(filename))
    
    def test_market_based_allocation(self):
        """Test market-based allocation method."""
        # Start market-based node
        self.market_thread.start()
        time.sleep(0.5)  # Wait for node to initialize
        
        # Reset subscriber
        self.assignment_subscriber.assignments = {}
        self.assignment_subscriber.update_count = 0
        
        # Publish tasks
        self.publish_test_tasks()
        
        # Wait for allocation
        time.sleep(0.5)
        
        # Verify assignments were made
        self.assertGreater(self.assignment_subscriber.update_count, 0)
        self.assertEqual(len(self.assignment_subscriber.assignments), 10)
        
        # Verify results file was created
        filename = os.path.join(self.temp_dir, 'comparative_MARKET_BASED_T10.json')
        self.assertTrue(os.path.exists(filename))
    
    def test_sequential_allocation(self):
        """Test sequential allocation method."""
        # Start sequential node
        self.sequential_thread.start()
        time.sleep(0.5)  # Wait for node to initialize
        
        # Reset subscriber
        self.assignment_subscriber.assignments = {}
        self.assignment_subscriber.update_count = 0
        
        # Publish tasks
        self.publish_test_tasks()
        
        # Wait for allocation
        time.sleep(0.5)
        
        # Verify assignments were made
        self.assertGreater(self.assignment_subscriber.update_count, 0)
        self.assertEqual(len(self.assignment_subscriber.assignments), 10)
        
        # Verify pattern of sequential assignment
        # Tasks should alternate between robots
        for i in range(1, 11):
            expected_robot = (i % 2) + 1
            self.assertEqual(self.assignment_subscriber.assignments.get(i), expected_robot,
                            f"Task {i} should be assigned to robot {expected_robot}")
        
        # Verify results file was created
        filename = os.path.join(self.temp_dir, 'comparative_SEQUENTIAL_T10.json')
        self.assertTrue(os.path.exists(filename))

if __name__ == '__main__':
    unittest.main()