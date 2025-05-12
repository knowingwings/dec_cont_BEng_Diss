# comparative_analysis.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time
from enum import Enum
import json
import os

from decentralized_control.msg import Task, TaskList, TaskAssignment

class AllocationMethod(Enum):
    """Enumeration of the allocation methods used for comparison."""
    CENTRALIZED = 1
    GREEDY = 2
    MARKET_BASED = 3
    SEQUENTIAL = 4

class ComparativeAnalysis(Node):
    """
    Implements alternative allocation approaches for comparison with the distributed auction algorithm.
    """
    
    def __init__(self):
        super().__init__('comparative_analysis')
        
        # Initialize parameters
        self.declare_parameter('output_dir', '/tmp/decentralized_control/results')
        self.declare_parameter('allocation_method', 'CENTRALIZED')
        self.declare_parameter('num_tasks', 10)
        self.declare_parameter('num_robots', 2)
        
        # Get parameters
        self.output_dir = self.get_parameter('output_dir').value
        self.allocation_method_str = self.get_parameter('allocation_method').value
        self.num_tasks = self.get_parameter('num_tasks').value
        self.num_robots = self.get_parameter('num_robots').value
        
        # Parse allocation method
        try:
            self.allocation_method = AllocationMethod[self.allocation_method_str]
        except KeyError:
            self.get_logger().error(f'Invalid allocation method: {self.allocation_method_str}')
            self.allocation_method = AllocationMethod.CENTRALIZED
        
        # Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Initialize state
        self.tasks = {}  # Dict of task_id -> Task
        self.robot_capabilities = {}
        self.robot_positions = {}
        for i in range(1, self.num_robots + 1):
            self.robot_capabilities[i] = np.ones(5)  # Default capabilities
            self.robot_positions[i] = np.zeros(3)    # Default position
        
        # Publishers
        self.assignment_publisher = self.create_publisher(
            TaskAssignment, '/comparative/assignments', 10)
        
        # Subscribers
        self.task_subscriber = self.create_subscription(
            TaskList, '/tasks', self.task_callback, 10)
        
        # Timer for allocation
        self.allocation_timer = self.create_timer(1.0, self.allocation_callback)
        
        self.get_logger().info(
            f'Comparative analysis initialized with method: {self.allocation_method_str}')
    
    def task_callback(self, msg):
        """Process incoming tasks."""
        for task in msg.tasks:
            self.tasks[task.id] = task
    
    def allocation_callback(self):
        """Perform allocation based on selected method."""
        if not self.tasks:
            return
        
        start_time = time.time()
        
        # Choose allocation method
        if self.allocation_method == AllocationMethod.CENTRALIZED:
            assignments = self.centralized_allocation()
        elif self.allocation_method == AllocationMethod.GREEDY:
            assignments = self.greedy_allocation()
        elif self.allocation_method == AllocationMethod.MARKET_BASED:
            assignments = self.market_based_allocation()
        elif self.allocation_method == AllocationMethod.SEQUENTIAL:
            assignments = self.sequential_allocation()
        else:
            self.get_logger().error(f'Unknown allocation method: {self.allocation_method}')
            return
        
        # Compute allocation time
        allocation_time = time.time() - start_time
        
        # Publish assignment
        self.publish_assignment(assignments)
        
        # Save results
        self.save_results(assignments, allocation_time)
    
    def centralized_allocation(self):
        """
        Implement centralized optimal allocation.
        Uses a simplified version of the Hungarian algorithm for minimum makespan.
        """
        # Calculate cost matrix (estimated execution time including travel)
        cost_matrix = np.zeros((self.num_robots, len(self.tasks)))
        
        task_ids = list(self.tasks.keys())
        
        for i in range(self.num_robots):
            robot_id = i + 1
            robot_pos = np.array(self.robot_positions[robot_id])
            
            for j, task_id in enumerate(task_ids):
                task = self.tasks[task_id]
                task_pos = np.array(task.position)
                
                # Calculate distance
                distance = np.linalg.norm(robot_pos - task_pos)
                
                # Calculate travel time (simplified)
                travel_time = distance / 0.5  # Assuming 0.5 m/s speed
                
                # Total time is travel time + execution time
                total_time = travel_time + task.execution_time
                
                cost_matrix[i, j] = total_time
        
        # Use a simplified version of the Hungarian algorithm
        # In a real implementation, would use scipy.optimize.linear_sum_assignment
        
        # For this example, just use a greedy approach to minimize makespan
        assignments = {}  # task_id -> robot_id
        robot_makespans = np.zeros(self.num_robots)
        
        # Sort tasks by execution time (longest first)
        sorted_indices = sorted(range(len(task_ids)), 
                              key=lambda j: self.tasks[task_ids[j]].execution_time,
                              reverse=True)
        
        for j in sorted_indices:
            task_id = task_ids[j]
            
            # Assign to robot with minimum current makespan
            min_makespan_idx = np.argmin(robot_makespans)
            robot_id = min_makespan_idx + 1
            
            # Update makespan
            robot_makespans[min_makespan_idx] += cost_matrix[min_makespan_idx, j]
            
            # Record assignment
            assignments[task_id] = robot_id
        
        return assignments
    
    def greedy_allocation(self):
        """
        Implement greedy allocation based on distance.
        Each task is assigned to the closest available robot.
        """
        assignments = {}  # task_id -> robot_id
        robot_workloads = {i: 0.0 for i in range(1, self.num_robots + 1)}
        
        # Sort tasks by ID (simple sequential allocation)
        task_ids = sorted(self.tasks.keys())
        
        for task_id in task_ids:
            task = self.tasks[task_id]
            task_pos = np.array(task.position)
            
            # Find closest robot
            min_distance = float('inf')
            closest_robot = None
            
            for robot_id in range(1, self.num_robots + 1):
                robot_pos = np.array(self.robot_positions[robot_id])
                
                # Calculate distance
                distance = np.linalg.norm(robot_pos - task_pos)
                
                # Update closest robot
                if distance < min_distance:
                    min_distance = distance
                    closest_robot = robot_id
            
            # Assign task to closest robot
            assignments[task_id] = closest_robot
            
            # Update robot workload
            robot_workloads[closest_robot] += task.execution_time
        
        return assignments
    
    def market_based_allocation(self):
        """
        Implement market-based allocation without consensus.
        Simple one-shot auction for all tasks.
        """
        assignments = {}  # task_id -> robot_id
        task_prices = {task_id: 0.0 for task_id in self.tasks.keys()}
        
        # Calculate bids for all robot-task pairs
        bids = {}  # (robot_id, task_id) -> bid_value
        
        for robot_id in range(1, self.num_robots + 1):
            robot_pos = np.array(self.robot_positions[robot_id])
            robot_cap = np.array(self.robot_capabilities[robot_id])
            
            for task_id, task in self.tasks.items():
                task_pos = np.array(task.position)
                
                # Calculate distance
                distance = np.linalg.norm(robot_pos - task_pos)
                
                # Simple bid based on distance (higher bid = better match)
                bid_value = 1.0 / (1.0 + distance)
                
                # Store bid
                bids[(robot_id, task_id)] = bid_value
        
        # Allocate tasks based on highest bid
        unassigned_tasks = list(self.tasks.keys())
        
        while unassigned_tasks:
            # Find best bid among unassigned tasks
            best_bid = -float('inf')
            best_robot = None
            best_task = None
            
            for task_id in unassigned_tasks:
                for robot_id in range(1, self.num_robots + 1):
                    bid_value = bids.get((robot_id, task_id), 0.0)
                    
                    if bid_value > best_bid:
                        best_bid = bid_value
                        best_robot = robot_id
                        best_task = task_id
            
            # Assign task to best robot
            if best_robot and best_task:
                assignments[best_task] = best_robot
                unassigned_tasks.remove(best_task)
            else:
                break
        
        return assignments
    
    def sequential_allocation(self):
        """
        Implement sequential allocation.
        Robots take turns picking tasks in order.
        """
        assignments = {}  # task_id -> robot_id
        
        # Sort tasks by ID
        task_ids = sorted(self.tasks.keys())
        
        # Assign tasks sequentially to robots
        for i, task_id in enumerate(task_ids):
            robot_id = (i % self.num_robots) + 1
            assignments[task_id] = robot_id
        
        return assignments
    
    def publish_assignment(self, assignments):
        """Publish task assignments."""
        msg = TaskAssignment()
        
        for task_id, robot_id in assignments.items():
            msg.task_ids.append(task_id)
            msg.robot_ids.append(robot_id)
            msg.prices.append(0.0)  # Not used in comparative methods
        
        self.assignment_publisher.publish(msg)
    
    def save_results(self, assignments, allocation_time):
        """Save allocation results for analysis."""
        # Calculate makespan for each robot
        robot_makespans = {i: 0.0 for i in range(1, self.num_robots + 1)}
        
        for task_id, robot_id in assignments.items():
            task = self.tasks[task_id]
            robot_makespans[robot_id] += task.execution_time
        
        # Calculate overall makespan
        makespan = max(robot_makespans.values())
        
        # Calculate load balancing metric
        if self.num_robots > 1:
            load_std = np.std(list(robot_makespans.values()))
            load_ratio = min(robot_makespans.values()) / max(robot_makespans.values()) if max(robot_makespans.values()) > 0 else 1.0
        else:
            load_std = 0.0
            load_ratio = 1.0
        
        # Results
        results = {
            'allocation_method': self.allocation_method_str,
            'num_tasks': self.num_tasks,
            'num_robots': self.num_robots,
            'allocation_time': allocation_time,
            'makespan': makespan,
            'robot_makespans': robot_makespans,
            'load_std': load_std,
            'load_ratio': load_ratio,
            'assignments': {str(k): v for k, v in assignments.items()}
        }
        
        # Save to file
        filename = os.path.join(
            self.output_dir, 
            f'comparative_{self.allocation_method_str}_T{self.num_tasks}.json')
        
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        
        self.get_logger().info(
            f'Comparative results saved to {filename}: '
            f'makespan={makespan:.2f}, allocation_time={allocation_time:.4f}s')