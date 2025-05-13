# task_dependency_analyzer.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from collections import defaultdict, deque

from dec_control.msg import Task, TaskList, TaskPriority

class TaskDependencyAnalyzer(Node):
    """
    Analyzes task dependencies, computes critical paths, and dynamic priorities.
    """
    
    def __init__(self):
        super().__init__('task_dependency_analyzer')
        
        # Parameters
        self.declare_parameter('priority_weights', [0.4, 0.3, 0.2, 0.1])  # weight for depth, critical, urgent, dependencies
        
        # Get parameters
        self.priority_weights = self.get_parameter('priority_weights').value
        
        # Initialize task data
        self.tasks = {}  # Dict of task_id -> Task
        self.completion_status = {}  # Dict of task_id -> completion status
        self.task_graph = defaultdict(list)  # Adjacency list representation of dependency graph
        self.reverse_graph = defaultdict(list)  # Reverse graph for backward pass
        
        # Critical path analysis results
        self.earliest_start = {}  # Dict of task_id -> earliest start time
        self.earliest_finish = {}  # Dict of task_id -> earliest finish time
        self.latest_start = {}  # Dict of task_id -> latest start time
        self.latest_finish = {}  # Dict of task_id -> latest finish time
        self.slack = {}  # Dict of task_id -> slack time
        self.critical_path = []  # List of task IDs on critical path
        
        # Dynamic priorities
        self.task_priorities = {}  # Dict of task_id -> priority value
        
        # Publishers
        self.priority_publisher = self.create_publisher(
            TaskPriority, '/task_priorities', 10)
        
        # Subscribers
        self.task_subscriber = self.create_subscription(
            TaskList, '/tasks', self.task_callback, 10)
        
        # Timer for periodic analysis
        self.analysis_timer = self.create_timer(1.0, self.analyze_dependencies)
        
        self.get_logger().info('Task dependency analyzer initialized')
    
    def task_callback(self, msg):
        """Process incoming tasks."""
        for task in msg.tasks:
            self.tasks[task.id] = task
            self.completion_status[task.id] = 0  # Not completed
            
            # Update dependency graph
            self.task_graph[task.id] = []
            for prereq in task.prerequisites:
                self.task_graph[prereq].append(task.id)
                self.reverse_graph[task.id].append(prereq)
        
        # Trigger immediate analysis
        self.analyze_dependencies()
    
    def analyze_dependencies(self):
        """Analyze task dependencies and compute critical path."""
        if not self.tasks:
            return
        
        # Forward pass - calculate earliest start and finish times
        self.compute_earliest_times()
        
        # Backward pass - calculate latest start and finish times
        self.compute_latest_times()
        
        # Calculate slack and identify critical path
        self.compute_slack_and_critical_path()
        
        # Calculate dynamic priorities
        self.compute_dynamic_priorities()
        
        # Publish priorities
        self.publish_priorities()
    
    def compute_earliest_times(self):
        """Compute earliest start and finish times (forward pass)."""
        # Initialize all earliest times to 0
        self.earliest_start = {task_id: 0 for task_id in self.tasks}
        self.earliest_finish = {task_id: 0 for task_id in self.tasks}
        
        # Find tasks with no prerequisites (roots)
        root_tasks = [task_id for task_id, task in self.tasks.items() if not task.prerequisites]
        
        # Initialize queue with root tasks
        queue = deque(root_tasks)
        visited = set()
        
        while queue:
            task_id = queue.popleft()
            
            if task_id in visited:
                continue
                
            visited.add(task_id)
            
            # Get the task
            task = self.tasks[task_id]
            
            # Calculate earliest start time (max of all prerequisites' finish times)
            if task.prerequisites:
                self.earliest_start[task_id] = max(
                    self.earliest_finish.get(prereq, 0) for prereq in task.prerequisites)
            else:
                self.earliest_start[task_id] = 0
            
            # Calculate earliest finish time
            self.earliest_finish[task_id] = self.earliest_start[task_id] + task.execution_time
            
            # Add successors to queue
            for successor in self.task_graph[task_id]:
                # Only add successor if all its prerequisites have been visited
                if all(prereq in visited for prereq in self.tasks[successor].prerequisites):
                    queue.append(successor)
    
    def compute_latest_times(self):
        """Compute latest start and finish times (backward pass)."""
        # Find project completion time (max of all earliest finish times)
        project_completion = max(self.earliest_finish.values())
        
        # Initialize all latest times to project completion
        self.latest_start = {task_id: project_completion for task_id in self.tasks}
        self.latest_finish = {task_id: project_completion for task_id in self.tasks}
        
        # Find tasks with no successors (leaf nodes)
        leaf_tasks = [task_id for task_id in self.tasks if not self.task_graph[task_id]]
        
        # Initialize queue with leaf tasks
        queue = deque(leaf_tasks)
        visited = set()
        
        while queue:
            task_id = queue.popleft()
            
            if task_id in visited:
                continue
                
            visited.add(task_id)
            
            # Get the task
            task = self.tasks[task_id]
            
            # For leaf tasks, latest finish = project completion
            if not self.task_graph[task_id]:
                self.latest_finish[task_id] = project_completion
            else:
                # Latest finish is min of all successors' latest start
                self.latest_finish[task_id] = min(
                    self.latest_start.get(succ, project_completion) 
                    for succ in self.task_graph[task_id])
            
            # Calculate latest start time
            self.latest_start[task_id] = self.latest_finish[task_id] - task.execution_time
            
            # Add predecessors to queue
            for prereq in self.reverse_graph[task_id]:
                queue.append(prereq)
    
    def compute_slack_and_critical_path(self):
        """Compute slack times and identify critical path."""
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
        
        self.get_logger().info(f'Critical path: {self.critical_path}')
    
    def compute_dynamic_priorities(self):
        """Compute dynamic priorities for tasks."""
        self.task_priorities = {}
        
        # Get maximum depth in the task graph
        max_depth = self.compute_max_depth()
        
        for task_id in self.tasks:
            # Skip completed tasks
            if self.completion_status.get(task_id, 0) == 1:
                continue
            
            # Calculate priority components
            depth = self.compute_task_depth(task_id) / max(1, max_depth)  # Normalized depth
            critical = 1.0 if task_id in self.critical_path else 0.0
            
            # Urgency based on slack (normalized inverse)
            max_slack = max(self.slack.values()) if self.slack else 1.0
            urgent = 1.0 - (self.slack[task_id] / max_slack) if max_slack > 0 else 0.0
            
            # Number of dependent tasks (normalized)
            dependents = len(self.task_graph[task_id])
            max_dependents = max(len(deps) for deps in self.task_graph.values()) if self.task_graph else 1
            dependencies = dependents / max(1, max_dependents)
            
            # Calculate weighted priority
            priority = (self.priority_weights[0] * depth +
                        self.priority_weights[1] * critical +
                        self.priority_weights[2] * urgent +
                        self.priority_weights[3] * dependencies)
            
            self.task_priorities[task_id] = priority
    
    def compute_max_depth(self):
        """Compute maximum depth in the task graph."""
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
    
    def compute_task_depth(self, task_id):
        """Compute depth of a task in the dependency graph."""
        if not self.reverse_graph[task_id]:  # No prerequisites
            return 0
        
        max_prereq_depth = 0
        for prereq in self.reverse_graph[task_id]:
            max_prereq_depth = max(max_prereq_depth, self.compute_task_depth(prereq))
        
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