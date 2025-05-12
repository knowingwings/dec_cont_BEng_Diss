#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
import time
from threading import Thread

from decentralized_control.task_manager.task_manager_node import TaskManagerNode
from decentralized_control.msg import TaskList, TaskAssignment
from dec_control.srv import GetAvailableTasks

class TaskListSubscriber(Node):
    """Test node for subscribing to task lists."""
    
    def __init__(self):
        super().__init__('task_list_subscriber')
        self.subscription = self.create_subscription(
            TaskList, '/tasks', self.task_callback, 10)
        self.tasks = {}
        self.update_count = 0
    
    def task_callback(self, msg):
        for task in msg.tasks:
            self.tasks[task.id] = task
        
        self.update_count += 1

class TestTaskManager(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.task_manager = TaskManagerNode()
        
        # Override parameters
        cls.task_manager.num_tasks = 10
        cls.task_manager.env_size = [4.0, 4.0, 2.0]
        cls.task_manager.dependency_prob = 0.3
        
        # Create subscriber node
        cls.task_subscriber = TaskListSubscriber()
        
        # Create assignment publisher node
        cls.assignment_pub = Node('assignment_publisher')
        cls.assignment_publisher = cls.assignment_pub.create_publisher(
            TaskAssignment, '/auction/assignments', 10)
        
        # Create client for GetAvailableTasks service
        cls.available_tasks_client = Node('available_tasks_client')
        cls.client = cls.available_tasks_client.create_client(
            GetAvailableTasks, '/task_manager/get_available_tasks')
        
        # Start spinning in separate threads
        cls.manager_thread = Thread(target=lambda: rclpy.spin(cls.task_manager))
        cls.subscriber_thread = Thread(target=lambda: rclpy.spin(cls.task_subscriber))
        cls.pub_thread = Thread(target=lambda: rclpy.spin(cls.assignment_pub))
        cls.client_thread = Thread(target=lambda: rclpy.spin(cls.available_tasks_client))
        
        cls.manager_thread.daemon = True
        cls.subscriber_thread.daemon = True
        cls.pub_thread.daemon = True
        cls.client_thread.daemon = True
        
        cls.manager_thread.start()
        cls.subscriber_thread.start()
        cls.pub_thread.start()
        cls.client_thread.start()
        
        # Wait for nodes to initialize
        time.sleep(1)
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def test_task_creation(self):
        """Test that tasks are created correctly."""
        # Tasks should have been created during initialization
        self.assertEqual(len(self.task_manager.tasks), 10)
        
        # Tasks should have been published
        self.assertGreater(self.task_subscriber.update_count, 0)
        self.assertEqual(len(self.task_subscriber.tasks), 10)
        
        # Verify task properties
        for task_id, task in self.task_subscriber.tasks.items():
            # ID should be between 1 and 10
            self.assertGreaterEqual(task_id, 1)
            self.assertLessEqual(task_id, 10)
            
            # Position should be within environment bounds
            self.assertLessEqual(task.position[0], self.task_manager.env_size[0])
            self.assertLessEqual(task.position[1], self.task_manager.env_size[1])
            self.assertLessEqual(task.position[2], self.task_manager.env_size[2])
            
            # Execution time should be positive
            self.assertGreater(task.execution_time, 0)
            
            # Capabilities should be normalized
            capabilities = task.capabilities_required
            self.assertGreaterEqual(len(capabilities), 1)
    
    def test_task_dependencies(self):
        """Test that task dependencies form a valid DAG."""
        # Check for cycles in the dependency graph
        visited = set()
        temp_visited = set()
        
        def has_cycle(task_id):
            if task_id in temp_visited:
                return True
            
            if task_id in visited:
                return False
            
            temp_visited.add(task_id)
            
            task = self.task_manager.tasks[task_id]
            for prereq in task.prerequisites:
                if has_cycle(prereq):
                    return True
            
            temp_visited.remove(task_id)
            visited.add(task_id)
            return False
        
        # Check all tasks for cycles
        for task_id in self.task_manager.tasks:
            self.assertFalse(has_cycle(task_id), f"Found cycle involving task {task_id}")
    
    def test_available_tasks(self):
        """Test that available tasks are correctly determined."""
        # Get available tasks using service
        request = GetAvailableTasks.Request()
        
        # Wait for service to be available
        self.assertTrue(self.client.wait_for_service(timeout_sec=1.0))
        
        future = self.client.call_async(request)
        
        # Wait for response
        timeout = 1.0  # seconds
        start_time = time.time()
        while not future.done() and time.time() - start_time < timeout:
            time.sleep(0.1)
        
        self.assertTrue(future.done(), "Service call timed out")
        
        # Get response
        response = future.result()
        
        # Verify available tasks
        available_task_ids = response.task_ids
        
        # All tasks without prerequisites should be available
        for task_id, task in self.task_manager.tasks.items():
            if not task.prerequisites:
                self.assertIn(task_id, available_task_ids, 
                             f"Task {task_id} without prerequisites should be available")
    
    def test_task_completion(self):
        """Test that task completion updates available tasks."""
        # Mark initial available tasks
        initial_available = self.task_manager.available_tasks.copy()
        
        # Find a task with prerequisites
        dependent_task = None
        prerequisite_task = None
        
        for task_id, task in self.task_manager.tasks.items():
            if task.prerequisites:
                dependent_task = task_id
                prerequisite_task = task.prerequisites[0]
                break
        
        if dependent_task is None:
            self.skipTest("No task with prerequisites found")
        
        # Verify dependent task is not available
        self.assertNotIn(dependent_task, self.task_manager.available_tasks)
        
        # Mark prerequisite task as completed
        self.task_manager.completion_status[prerequisite_task] = 1
        
        # Update available tasks
        self.task_manager.update_available_tasks()
        
        # Check if dependent task is now available
        # This will be true if dependent_task has no other prerequisites
        all_prereqs_completed = all(
            self.task_manager.completion_status.get(prereq, 0) == 1 
            for prereq in self.task_manager.tasks[dependent_task].prerequisites)
        
        if all_prereqs_completed:
            self.assertIn(dependent_task, self.task_manager.available_tasks,
                         f"Task {dependent_task} should be available after prerequisites completed")

if __name__ == '__main__':
    unittest.main()