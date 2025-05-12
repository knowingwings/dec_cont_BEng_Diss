# decentralized_control/task_manager/critical_path_analyzer.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from collections import defaultdict, deque
import time
from threading import Lock

from decentralized_control.msg import Task, TaskList, TaskPriority, TaskAssignment

class CriticalPathAnalyzer(Node):
    """
    Analyzes task dependencies using Critical Path Method (CPM)
    to identify critical paths and calculate task priorities.
    """
    
    def __init__(self):
        super().__init__('critical_path_analyzer')
        
        # Parameters
        self.declare_parameter('priority_weights', [0.4, 0.3, 0.2, 0.1])  # depth, critical, urgent, dependencies
        self.declare_parameter('update_interval', 1.0)  # seconds
        
        # Get parameters
        self.priority_weights = self.get_parameter('priority_weights').value
        self.update_interval = self.get_parameter('update_interval').value
        
        # Initialize state
        self.tasks = {}  # task_id -> Task
        self.task_graph = defaultdict(list)  # task_id -> list of successor tasks
        self.reverse_graph = defaultdict(list)  # task_id -> list of prerequisite tasks
        self.completion_status = {}  # task_id -> completion status
        
        # CPM analysis results
        self.earliest_start = {}  # task_id -> earliest start time
        self.earliest_finish = {}  # task_id -> earliest finish time
        self.latest_start = {}  # task_id -> latest start time
        self.latest_finish = {}  # task_id -> latest finish time
        self.slack = {}  # task_id -> slack time
        self.critical_path = []  # list of task IDs on critical path
        self.task_priorities = {}  # task_id -> priority value
        
        # Lock for thread safety
        self.lock = Lock()
        
        # Publishers
        self.priority_publisher = self.create_publisher(
            TaskPriority, '/task_priorities', 10)
        
        # Subscribers
        self.task_subscriber = self.create_subscription(
            TaskList, '/tasks', self.task_callback, 10)
            
        self.assignment_subscriber = self.create_subscription(
            TaskAssignment, '/auction/assignments', self.assignment_callback, 10)
        
        # Timer for periodic analysis
        self.analysis_timer = self.create_timer(
            self.update_interval, self.analyze_critical_path)
        
        self.get_logger().info('Critical path analyzer initialized')
    
    def task_callback(self, msg):
        """Process incoming tasks and build task dependency graph."""
        with self.lock:
            # Update task information
            for task in msg.tasks:
                self.tasks[task.id] = task
                self.completion_status[task.id] = 0  # Not completed
                
                # Clear existing graph entries
                self.task_graph[task.id] = []
                self.reverse_graph[task.id] = []
            
            # Build dependency graph
            for task_id, task in self.tasks.items():
                # Add prerequisites
                self.reverse_graph[task_id] = task.prerequisites
                
                # Add task as successor to its prerequisites
                for prereq in task.prerequisites:
                    self.task_graph[prereq].append(task_id)
            
            self.get_logger().info(
                f'Task graph updated with {len(self.tasks)} tasks')
    
    def assignment_callback(self, msg):
        """Process task assignments to update completion status."""
        with self.lock:
            # For this implementation, we'll assume a task is completed
            # when it's assigned to a robot (in a real implementation,
            # would track actual completion)
            for i in range(len(msg.task_ids)):
                task_id = msg.task_ids[i]
                robot_id = msg.robot_ids[i]
                
                if task_id in self.completion_status and robot_id > 0:
                    self.completion_status[task_id] = 1  # Completed
    
    def analyze_critical_path(self):
        """Apply Critical Path Method to analyze task dependencies."""
        with self.lock:
            if not self.tasks:
                return
            
            # Forward pass - calculate earliest start and finish times
            self.forward_pass()
            
            # Backward pass - calculate latest start and finish times
            self.backward_pass()
            
            # Calculate slack and identify critical path
            self.calculate_slack_and_critical_path()
            
            # Calculate dynamic priorities
            self.calculate_dynamic_priorities()
            
            # Publish priorities
            self.publish_priorities()
            
            self.get_logger().info(
                f'Critical path analysis completed, identified {len(self.critical_path)} critical tasks')
    
    def forward_pass(self):
        """
        Forward pass of Critical Path Method.
        Calculates earliest start and finish times.
        """
        # Initialize earliest times
        self.earliest_start = {task_id: 0 for task_id in self.tasks}
        self.earliest_finish = {task_id: 0 for task_id in self.tasks}
        
        # Find tasks with no prerequisites (roots)
        root_tasks = [task_id for task_id, task in self.tasks.items() 
                      if not self.reverse_graph[task_id]]
        
        # Process tasks in topological order
        visited = set()
        queue = deque(root_tasks)
        
        while queue:
            task_id = queue.popleft()
            
            if task_id in visited:
                continue
                
            # Check if all prerequisites have been visited
            prereqs = self.reverse_graph[task_id]
            if not all(prereq in visited for prereq in prereqs):
                # Put back in queue and continue
                queue.append(task_id)
                continue
            
            # Mark as visited
            visited.add(task_id)
            
            # Calculate earliest start time (max of all prerequisites' finish times)
            if prereqs:
                self.earliest_start[task_id] = max(
                    self.earliest_finish.get(prereq, 0) for prereq in prereqs)
            else:
                self.earliest_start[task_id] = 0
            
            # Calculate earliest finish time
            task = self.tasks[task_id]
            self.earliest_finish[task_id] = self.earliest_start[task_id] + task.execution_time
            
            # Add successors to queue
            for successor in self.task_graph[task_id]:
                queue.append(successor)
    
    def backward_pass(self):
        """
        Backward pass of Critical Path Method.
        Calculates latest start and finish times.
        """
        # Find project completion time (max of all earliest finish times)
        project_completion = max(self.earliest_finish.values()) if self.earliest_finish else 0
        
        # Initialize latest times to project completion
        self.latest_start = {task_id: project_completion for task_id in self.tasks}
        self.latest_finish = {task_id: project_completion for task_id in self.tasks}
        
        # Find tasks with no successors (leaf nodes)
        leaf_tasks = [task_id for task_id in self.tasks if not self.task_graph[task_id]]
        
        # Process tasks in reverse topological order
        visited = set()
        queue = deque(leaf_tasks)
        
        while queue:
            task_id = queue.popleft()
            
            if task_id in visited:
                continue
                
            # Check if all successors have been visited
            successors = self.task_graph[task_id]
            if not all(succ in visited for succ in successors):
                # Put back in queue and continue
                queue.append(task_id)
                continue
            
            # Mark as visited
            visited.add(task_id)
            
            # For leaf tasks, latest finish = project completion
            if not successors:
                self.latest_finish[task_id] = project_completion
            else:
                # Latest finish is min of all successors' latest start
                self.latest_finish[task_id] = min(
                    self.latest_start.get(succ, project_completion) 
                    for succ in successors)
            
            # Calculate latest start time
            task = self.tasks[task_id]
            self.latest_start[task_id] = self.latest_finish[task_id] - task.execution_time
            
            # Add prerequisites to queue
            for prereq in self.reverse_graph[task_id]:
                queue.append(prereq)
    
    def calculate_slack_and_critical_path(self):
        """Calculate slack times and identify critical path."""
        self.slack = {}
        self.critical_path = []
        
        for task_id in self.tasks:
            # Calculate slack
            self.slack[task_id] = self.latest_start[task_id] - self.earliest_start[task_id]
            
            # Tasks with zero slack are on critical path
            if abs(self.slack[task_id]) < 0.001:  # Use small epsilon for float comparison
                self.critical_path.append(task_id)
        
        # Sort critical path by earliest start time
        self.critical_path.sort(key=lambda task_id: self.earliest_start[task_id])
    
    def calculate_dynamic_priorities(self):
        """Calculate dynamic priorities for tasks based on multiple factors."""
        self.task_priorities = {}
        
        # Get maximum depth in the task graph
        max_depth = self.calculate_max_depth()
        
        # Calculate priorities for each task
        for task_id in self.tasks:
            # Skip completed tasks
            if self.completion_status.get(task_id, 0) == 1:
                continue
            
            # Calculate priority components
            depth = self.calculate_task_depth(task_id) / max(1, max_depth)  # Normalized depth
            critical = 1.0 if task_id in self.critical_path else 0.0
            
            # Urgency based on slack (normalized inverse)
            max_slack = max(self.slack.values()) if self.slack else 1.0
            urgent = 1.0 - (self.slack[task_id] / max_slack) if max_slack > 0 else 0.0
            
            # Number of dependent tasks (normalized)
            dependent_count = len(self.task_graph[task_id])
            max_dependent_count = max(len(deps) for deps in self.task_graph.values()) if self.task_graph else 1
            dependencies = dependent_count / max(1, max_dependent_count)
            
            # Calculate weighted priority
            priority = (self.priority_weights[0] * depth +
                        self.priority_weights[1] * critical +
                        self.priority_weights[2] * urgent +
                        self.priority_weights[3] * dependencies)
            
            self.task_priorities[task_id] = priority
    
    def calculate_max_depth(self):
        """Calculate maximum depth in the task dependency graph."""
        depths = {}
        
        def dfs(task_id):
            if task_id in depths:
                return depths[task_id]
            
            if not self.reverse_graph[task_id]:  # No prerequisites
                depths[task_id] = 0
                return 0
            
            max_prereq_depth = 0
            for prereq in self.reverse_graph[task_id]:
                max_prereq_depth = max(max_prereq_depth, dfs(prereq))
            
            depths[task_id] = max_prereq_depth + 1
            return depths[task_id]
        
        max_depth = 0
        for task_id in self.tasks:
            max_depth = max(max_depth, dfs(task_id))
        
        return max_depth
    
    def calculate_task_depth(self, task_id):
        """Calculate depth of a task in the dependency graph."""
        if not self.reverse_graph[task_id]:  # No prerequisites
            return 0
        
        max_prereq_depth = 0
        for prereq in self.reverse_graph[task_id]:
            max_prereq_depth = max(max_prereq_depth, self.calculate_task_depth(prereq))
        
        return max_prereq_depth + 1
    
    def publish_priorities(self):
        """Publish task priorities."""
        if not self.task_priorities:
            return
        
        # Create message
        msg = TaskPriority()
        
        for task_id, priority in self.task_priorities.items():
            msg.task_ids.append(task_id)
            msg.priorities.append(priority)
            msg.on_critical_path.append(task_id in self.critical_path)
            msg.slack_times.append(self.slack.get(task_id, 0.0))
        
        self.priority_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CriticalPathAnalyzer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()