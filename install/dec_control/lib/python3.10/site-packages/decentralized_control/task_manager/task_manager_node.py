#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time
from threading import Lock

from decentralized_control.msg import Task, TaskList, TaskAssignment
from decentralized_control.srv import GetAvailableTasks

class TaskManagerNode(Node):
    """
    Manages task information, dependencies, and availability.
    """
    
    def __init__(self):
        super().__init__('task_manager_node')
        
        # Add lock for thread safety
        self.lock = Lock()
        
        # Initialize parameters
        self.declare_parameter('num_tasks', 10)
        self.declare_parameter('environment_size', [4.0, 4.0, 2.0])
        self.declare_parameter('dependency_probability', 0.3)
        
        # Get parameters
        self.num_tasks = self.get_parameter('num_tasks').value
        self.env_size = self.get_parameter('environment_size').value
        self.dependency_prob = self.get_parameter('dependency_probability').value
        
        # Initialize tasks
        self.tasks = {}  # Dict of task_id -> Task
        self.completion_status = {}  # Dict of task_id -> completion status
        self.available_tasks = []  # List of available task IDs
        
        # Publishers
        self.task_publisher = self.create_publisher(
            TaskList, '/tasks', 10)
        
        # Subscribers
        self.assignment_subscriber = self.create_subscription(
            TaskAssignment, '/auction/assignments', self.assignment_callback, 10)
        
        # Services
        self.get_available_tasks_service = self.create_service(
            GetAvailableTasks, '/task_manager/get_available_tasks', 
            self.get_available_tasks_callback)
        
        # Timer for periodic task updates
        self.task_update_timer = self.create_timer(
            0.5, self.task_update_callback)  # 2Hz updates
        
        # Create initial tasks
        self.create_tasks()
        
        self.get_logger().info(
            f'Task manager node initialized with {self.num_tasks} tasks')
    
    def create_tasks(self):
        """Create initial task set"""
        # Task capability patterns for different task types
        capability_patterns = [
            [0.8, 0.4, 0.6, 0.2, 0.3],  # Type 1: Strong in capabilities 1, 3
            [0.3, 0.9, 0.2, 0.8, 0.4],  # Type 2: Strong in capabilities 2, 4
            [0.5, 0.5, 0.7, 0.5, 0.9]   # Type 3: Strong in capability 5, balanced otherwise
        ]
        
        # Create tasks with more even distribution
        for i in range(1, self.num_tasks + 1):
            task = Task()
            task.id = i
            
            # Assign position with better distribution
            grid_size = int(np.ceil(np.sqrt(self.num_tasks * 1.5)))
            x_offset = self.env_size[0] / grid_size
            y_offset = self.env_size[1] / grid_size
            z_offset = self.env_size[2] / 4
            
            grid_x = (i - 1) % grid_size
            grid_y = (i - 1) // grid_size
            
            # Add small random variation
            x_jitter = 0.1 * x_offset * (np.random.random() - 0.5)
            y_jitter = 0.1 * y_offset * (np.random.random() - 0.5)
            z_jitter = 0.1 * z_offset * (np.random.random() - 0.5)
            
            task.position = [
                x_offset * (0.5 + grid_x) + x_jitter,
                y_offset * (0.5 + grid_y) + y_jitter,
                z_offset + z_jitter
            ]
            
            # Assign capability requirements using one of the patterns with small variations
            pattern_idx = (i - 1) % len(capability_patterns)
            base_capabilities = capability_patterns[pattern_idx]
            
            # Add small random variations
            variation = 0.1 * (np.random.random(5) - 0.5)
            
            capabilities = np.array(base_capabilities) + variation
            
            # Normalize capabilities to unit length
            capabilities = capabilities / np.linalg.norm(capabilities)
            task.capabilities_required = capabilities.tolist()
            
            # Execution time based on capability pattern
            if pattern_idx == 0:
                task.execution_time = 5 + 3 * np.random.random()  # 5-8 time units
            elif pattern_idx == 1:
                task.execution_time = 7 + 3 * np.random.random()  # 7-10 time units
            else:
                task.execution_time = 6 + 4 * np.random.random()  # 6-10 time units
            
            # Initialize with no prerequisites
            task.prerequisites = []
            
            # Determine if this task requires collaboration
            # For simplicity, only make a small percentage collaborative
            task.requires_collaboration = np.random.random() < 0.1
            
            # Add to task dict
            self.tasks[task.id] = task
            self.completion_status[task.id] = 0  # Not completed
        
        # Add dependencies after all tasks are created
        self.add_task_dependencies()
        
        # Determine initially available tasks
        self.update_available_tasks()
        
        # Publish initial task list
        self.publish_tasks()
    
    def add_task_dependencies(self):
        """Add prerequisites to create a directed acyclic graph"""
        num_tasks = len(self.tasks)
        
        # Sort tasks by position (x-coordinate) to help prevent cycles
        task_ids = list(self.tasks.keys())
        task_positions = [self.tasks[task_id].position for task_id in task_ids]
        x_coords = [pos[0] for pos in task_positions]
        
        sorted_indices = np.argsort(x_coords)
        sorted_task_ids = [task_ids[i] for i in sorted_indices]
        
        # Add dependencies according to sorted order
        for j in range(1, len(sorted_task_ids)):
            task_id = sorted_task_ids[j]
            
            # Consider only tasks that come before in the sorted order
            potential_prereqs = sorted_task_ids[:j]
            
            # Limit the maximum number of prerequisites to avoid overconstraining
            max_prereqs = min(3, len(potential_prereqs))
            
            # Randomly select prerequisites with the given probability
            for i in range(len(potential_prereqs)):
                if np.random.random() < self.dependency_prob and len(self.tasks[task_id].prerequisites) < max_prereqs:
                    prereq_id = potential_prereqs[i]
                    
                    # Avoid adding duplicates
                    if prereq_id not in self.tasks[task_id].prerequisites:
                        self.tasks[task_id].prerequisites.append(prereq_id)
        
        # Remove transitive dependencies to simplify the graph
        for i in range(1, num_tasks + 1):
            if i not in self.tasks:
                continue
                
            prereqs = self.tasks[i].prerequisites.copy()
            
            # Remove transitive dependencies
            for j in prereqs:
                if j not in self.tasks:
                    continue
                    
                for k in self.tasks[j].prerequisites:
                    # If prereq j has its own prereq k, and k is also a direct prereq of i,
                    # then k is a transitive prereq and can be removed from i's direct prereqs
                    if k in prereqs:
                        self.tasks[i].prerequisites.remove(k)
    
    def update_available_tasks(self):
        """Update the list of available tasks based on prerequisites"""
        with self.lock:
            completed_tasks = [task_id for task_id, status in self.completion_status.items() if status == 1]
            
            # Find tasks with all prerequisites completed
            self.available_tasks = []
            for task_id, task in self.tasks.items():
                if self.completion_status[task_id] == 1:
                    continue  # Skip completed tasks
                
                # Check prerequisites
                if not task.prerequisites:
                    self.available_tasks.append(task_id)
                else:
                    # Check if all prerequisites are completed
                    if all(prereq in completed_tasks for prereq in task.prerequisites):
                        self.available_tasks.append(task_id)
    
    def publish_tasks(self):
        """Publish current task information"""
        msg = TaskList()
        msg.tasks = list(self.tasks.values())
        
        self.task_publisher.publish(msg)
    
    def assignment_callback(self, msg):
        """Process task assignment updates"""
        with self.lock:
            # Update completion status based on assignments
            for i in range(len(msg.task_ids)):
                task_id = msg.task_ids[i]
                robot_id = msg.robot_ids[i]
                
                if task_id in self.completion_status and robot_id > 0:
                    self.completion_status[task_id] = 1  # Completed
            
            # Update available tasks after status changes
            self.update_available_tasks()
    
    def get_available_tasks_callback(self, request, response):
        """Service to get currently available tasks"""
        with self.lock:
            response.task_ids = self.available_tasks.copy()
        return response
    
    def task_update_callback(self):
        """Periodic update of task information"""
        # This would monitor task execution progress
        # For now, we'll just periodically update available tasks
        self.update_available_tasks()
        
        # Publish updated task list
        self.publish_tasks()