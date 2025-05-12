#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
import time
import os
import json
from threading import Thread

from decentralized_control.metrics_collector import MetricsCollector
from decentralized_control.msg import Task, TaskList, TaskAssignment, Bid, RobotState, Heartbeat

class TestMetrics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        
        # Create temporary output directory
        cls.temp_dir = '/tmp/decentralized_control_test'
        os.makedirs(cls.temp_dir, exist_ok=True)
        
        # Create metrics collector
        cls.metrics_collector = MetricsCollector()
        
        # Override parameters
        cls.metrics_collector.output_dir = cls.temp_dir
        cls.metrics_collector.experiment_id = 'test_experiment'
        cls.metrics_collector.num_tasks = 5
        cls.metrics_collector.delay_ms = 50
        cls.metrics_collector.packet_loss = 0.1
        cls.metrics_collector.epsilon = 0.05
        
        # Create publisher nodes
        cls.task_pub = Node('task_publisher')
        cls.assignment_pub = Node('assignment_publisher')
        cls.bid_pub1 = Node('bid_publisher1')
        cls.bid_pub2 = Node('bid_publisher2')
        cls.heartbeat_pub1 = Node('heartbeat_publisher1')
        cls.heartbeat_pub2 = Node('heartbeat_publisher2')
        cls.state_pub1 = Node('state_publisher1')
        cls.state_pub2 = Node('state_publisher2')
        
        # Create publishers
        cls.task_publisher = cls.task_pub.create_publisher(
            TaskList, '/tasks', 10)
        cls.assignment_publisher = cls.assignment_pub.create_publisher(
            TaskAssignment, '/auction/assignments', 10)
        cls.bid_publisher1 = cls.bid_pub1.create_publisher(
            Bid, '/robot1/auction/bids', 10)
        cls.bid_publisher2 = cls.bid_pub2.create_publisher(
            Bid, '/robot2/auction/bids', 10)
        cls.heartbeat_publisher1 = cls.heartbeat_pub1.create_publisher(
            Heartbeat, '/robot1/heartbeat', 10)
        cls.heartbeat_publisher2 = cls.heartbeat_pub2.create_publisher(
            Heartbeat, '/robot2/heartbeat', 10)
        cls.state_publisher1 = cls.state_pub1.create_publisher(
            RobotState, '/robot1/state', 10)
        cls.state_publisher2 = cls.state_pub2.create_publisher(
            RobotState, '/robot2/state', 10)
        
        # Start spinning in separate threads
        cls.collector_thread = Thread(target=lambda: rclpy.spin(cls.metrics_collector))
        cls.task_thread = Thread(target=lambda: rclpy.spin(cls.task_pub))
        cls.assignment_thread = Thread(target=lambda: rclpy.spin(cls.assignment_pub))
        cls.bid1_thread = Thread(target=lambda: rclpy.spin(cls.bid_pub1))
        cls.bid2_thread = Thread(target=lambda: rclpy.spin(cls.bid_pub2))
        cls.heartbeat1_thread = Thread(target=lambda: rclpy.spin(cls.heartbeat_pub1))
        cls.heartbeat2_thread = Thread(target=lambda: rclpy.spin(cls.heartbeat_pub2))
        cls.state1_thread = Thread(target=lambda: rclpy.spin(cls.state_pub1))
        cls.state2_thread = Thread(target=lambda: rclpy.spin(cls.state_pub2))
        
        cls.collector_thread.daemon = True
        cls.task_thread.daemon = True
        cls.assignment_thread.daemon = True
        cls.bid1_thread.daemon = True
        cls.bid2_thread.daemon = True
        cls.heartbeat1_thread.daemon = True
        cls.heartbeat2_thread.daemon = True
        cls.state1_thread.daemon = True
        cls.state2_thread.daemon = True
        
        cls.collector_thread.start()
        cls.task_thread.start()
        cls.assignment_thread.start()
        cls.bid1_thread.start()
        cls.bid2_thread.start()
        cls.heartbeat1_thread.start()
        cls.heartbeat2_thread.start()
        cls.state1_thread.start()
        cls.state2_thread.start()
        
        # Wait for nodes to initialize
        time.sleep(1)
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def test_task_tracking(self):
        """Test that tasks are tracked correctly."""
        # Publish tasks
        task_list = TaskList()
        
        for i in range(1, 6):
            task = Task()
            task.id = i
            task.position = [float(i), 0.0, 0.0]
            task.capabilities_required = [1.0, 1.0, 1.0, 1.0, 1.0]
            task.execution_time = 5.0
            task.prerequisites = []
            task.requires_collaboration = False
            
            task_list.tasks.append(task)
        
        self.task_publisher.publish(task_list)
        
        # Wait for tasks to be processed
        time.sleep(0.1)
        
        # Verify tasks are tracked
        self.assertEqual(len(self.metrics_collector.task_completion_times), 5)
    
    def test_message_counting(self):
        """Test that messages are counted correctly."""
        # Get initial counts
        initial_bids = self.metrics_collector.message_counts['bids']
        initial_assignments = self.metrics_collector.message_counts['assignments']
        initial_heartbeats = self.metrics_collector.message_counts['heartbeats']
        initial_state_updates = self.metrics_collector.message_counts['state_updates']
        initial_total = self.metrics_collector.total_messages
        
        # Send test messages
        # Bid
        bid = Bid()
        bid.robot_id = 1
        bid.task_id = 1
        bid.bid_value = 1.0
        bid.utility = 0.5
        
        self.bid_publisher1.publish(bid)
        
        # Assignment
        assignment = TaskAssignment()
        assignment.task_ids = [1]
        assignment.robot_ids = [1]
        assignment.prices = [0.5]
        
        self.assignment_publisher.publish(assignment)
        
        # Heartbeat
        heartbeat = Heartbeat()
        heartbeat.robot_id = 1
        heartbeat.timestamp = int(time.time() * 1000)
        heartbeat.status = 0
        
        self.heartbeat_publisher1.publish(heartbeat)
        
        # State
        state = RobotState()
        state.id = 1
        state.position = [0.0, 0.0, 0.0]
        state.orientation = [0.0, 0.0, 0.0]
        state.capabilities = [1.0, 1.0, 1.0, 1.0, 1.0]
        state.workload = 0.0
        state.failed = False
        
        self.state_publisher1.publish(state)
        
        # Wait for messages to be processed
        time.sleep(0.1)
        
        # Verify message counts increased
        self.assertEqual(self.metrics_collector.message_counts['bids'], initial_bids + 1)
        self.assertEqual(self.metrics_collector.message_counts['assignments'], initial_assignments + 1)
        self.assertEqual(self.metrics_collector.message_counts['heartbeats'], initial_heartbeats + 1)
        self.assertEqual(self.metrics_collector.message_counts['state_updates'], initial_state_updates + 1)
        self.assertEqual(self.metrics_collector.total_messages, initial_total + 4)
    
    def test_convergence_tracking(self):
        """Test that convergence is tracked correctly."""
        # Reset convergence time
        self.metrics_collector.convergence_time = None
        
        # Ensure task_completion_times is populated
        if not self.metrics_collector.task_completion_times:
            task_list = TaskList()
            for i in range(1, 6):
                task = Task()
                task.id = i
                task_list.tasks.append(task)
            
            self.task_publisher.publish(task_list)
            time.sleep(0.1)
        
        # Create task assignments for all tasks
        assignment = TaskAssignment()
        
        for i in range(1, 6):
            assignment.task_ids.append(i)
            assignment.robot_ids.append(1 if i % 2 == 0 else 2)  # Alternate between robots
            assignment.prices.append(0.5)
        
        self.assignment_publisher.publish(assignment)
        
        # Wait for assignment to be processed
        time.sleep(0.1)
        
        # Verify convergence time was set
        self.assertIsNotNone(self.metrics_collector.convergence_time)
        self.assertGreater(self.metrics_collector.convergence_time, 0)
    
    def test_failure_tracking(self):
        """Test that robot failures are tracked."""
        # Reset recovery events
        self.metrics_collector.recovery_events = []
        
        # Send failure state
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
        
        # Verify failure was recorded
        self.assertEqual(len(self.metrics_collector.recovery_events), 1)
        self.assertEqual(self.metrics_collector.recovery_events[0]['event'], 'failure')
        self.assertEqual(self.metrics_collector.recovery_events[0]['robot_id'], 2)
    
    def test_metrics_saving(self):
        """Test that metrics are saved to file."""
        # Force metrics calculation
        self.metrics_collector.calculate_metrics()
        
        # Wait for file to be written
        time.sleep(0.1)
        
        # Check if file exists
        filename = os.path.join(
            self.temp_dir, 
            f'metrics_{self.metrics_collector.experiment_id}_T{self.metrics_collector.num_tasks}_D{self.metrics_collector.delay_ms}_'
            f'P{int(self.metrics_collector.packet_loss*100)}_E{int(self.metrics_collector.epsilon*100)}.json')
        
        self.assertTrue(os.path.exists(filename), f"Metrics file not found: {filename}")
        
        # Load and verify metrics
        with open(filename, 'r') as f:
            metrics = json.load(f)
        
        self.assertEqual(metrics['experiment_id'], 'test_experiment')
        self.assertEqual(metrics['parameters']['num_tasks'], 5)
        self.assertEqual(metrics['parameters']['delay_ms'], 50)
        self.assertAlmostEqual(metrics['parameters']['packet_loss'], 0.1)
        self.assertAlmostEqual(metrics['parameters']['epsilon'], 0.05)

if __name__ == '__main__':
    unittest.main()