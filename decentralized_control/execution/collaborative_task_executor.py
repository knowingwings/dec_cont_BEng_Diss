# collaborative_task_executor.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time
from threading import Lock

from dec_control.msg import Task, TaskList, TaskAssignment, CollaborationRequest, CollaborationResponse, SynchronizationSignal

class CollaborativeTaskExecutor(Node):
    """
    Implements leader-follower paradigm for collaborative task execution.
    """
    
    def __init__(self):
        super().__init__('collaborative_task_executor')
        
        # Parameters
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('leadership_weights', [0.6, 0.3, 0.1])  # distance, workload, capability_match
        self.declare_parameter('sync_timeout', 5.0)  # seconds
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.leadership_weights = self.get_parameter('leadership_weights').value
        self.sync_timeout = self.get_parameter('sync_timeout').value
        
        # Initialize state variables
        self.position = np.zeros(3)  # [x, y, z]
        self.capabilities = np.ones(5)  # Example capabilities
        self.workload = 0.0
        self.collaborative_tasks = []  # List of tasks requiring collaboration
        self.current_collab_task = None
        self.is_leader = False
        self.follower_id = 3 - self.robot_id  # For dual robot system: 1 or 2
        self.sync_points = {}  # Dict of task_id -> list of synchronization points
        self.sync_states = {}  # Dict of task_id -> current synchronization state
        self.current_sync_point = 0
        self.last_sync_time = 0
        self.lock = Lock()
        
        # Publishers
        self.collab_request_pub = self.create_publisher(
            CollaborationRequest, '/collaboration/requests', 10)
        
        self.collab_response_pub = self.create_publisher(
            CollaborationResponse, '/collaboration/responses', 10)
        
        self.sync_signal_pub = self.create_publisher(
            SynchronizationSignal, f'/robot{self.robot_id}/collaboration/sync', 10)
        
        # Subscribers
        self.task_sub = self.create_subscription(
            TaskList, '/tasks', self.task_callback, 10)
        
        self.assignment_sub = self.create_subscription(
            TaskAssignment, '/auction/assignments', self.assignment_callback, 10)
        
        self.collab_request_sub = self.create_subscription(
            CollaborationRequest, '/collaboration/requests', self.collab_request_callback, 10)
        
        self.collab_response_sub = self.create_subscription(
            CollaborationResponse, '/collaboration/responses', self.collab_response_callback, 10)
        
        self.sync_signal_sub = self.create_subscription(
            SynchronizationSignal, f'/robot{self.follower_id}/collaboration/sync', 
            self.sync_signal_callback, 10)
        
        # Timer for executing collaborative tasks
        self.execution_timer = self.create_timer(0.1, self.execution_step)
        
        self.get_logger().info(
            f'Collaborative task executor initialized for Robot {self.robot_id}')
    
    def task_callback(self, msg):
        """Process incoming tasks and identify collaborative ones."""
        with self.lock:
            for task in msg.tasks:
                if task.requires_collaboration:
                    # If not already in collaborative tasks list
                    if task.id not in [t.id for t in self.collaborative_tasks]:
                        self.collaborative_tasks.append(task)
                        
                        # Create synchronization points for this task
                        # In a real implementation, this would depend on task type
                        # Here we create evenly spaced sync points
                        num_sync_points = 4  # For example
                        self.sync_points[task.id] = [
                            i / num_sync_points for i in range(1, num_sync_points + 1)]
                        
                        self.get_logger().info(
                            f'Added collaborative task {task.id} with {num_sync_points} sync points')
    
    def assignment_callback(self, msg):
        """
        Process task assignments to determine if this robot is
        participating in collaborative tasks.
        """
        with self.lock:
            # Check if any collaborative tasks are assigned to this robot
            for i in range(len(msg.task_ids)):
                task_id = msg.task_ids[i]
                robot_id = msg.robot_ids[i]
                
                # Check if this task is in our collaborative tasks list
                collab_task_ids = [task.id for task in self.collaborative_tasks]
                
                if task_id in collab_task_ids and robot_id == self.robot_id:
                    # We're assigned to this collaborative task
                    if self.current_collab_task is None:
                        # Find the task object
                        for task in self.collaborative_tasks:
                            if task.id == task_id:
                                self.current_collab_task = task
                                self.sync_states[task_id] = 0  # Initialize sync state
                                self.current_sync_point = 0
                                
                                # Initiate leader selection
                                self.initiate_leader_selection(task_id)
                                break
    
    def initiate_leader_selection(self, task_id):
        """Initiate leader selection for a collaborative task."""
        # Calculate leadership score for this robot
        distance_factor = 1.0  # Placeholder - would use task position
        workload_factor = 1.0 / (1.0 + self.workload)
        capability_factor = 0.8  # Placeholder - would calculate match score
        
        leadership_score = (
            self.leadership_weights[0] * distance_factor +
            self.leadership_weights[1] * workload_factor +
            self.leadership_weights[2] * capability_factor
        )
        
        # Send collaboration request with leadership score
        request = CollaborationRequest()
        request.task_id = task_id
        request.robot_id = self.robot_id
        request.leadership_score = leadership_score
        request.timestamp = int(time.time() * 1000)
        
        self.collab_request_pub.publish(request)
        
        self.get_logger().info(
            f'Initiated leader selection for task {task_id}, '
            f'leadership score: {leadership_score:.2f}')
    
    def collab_request_callback(self, msg):
        """
        Process collaboration requests from other robots.
        Compare leadership scores to determine roles.
        """
        if msg.robot_id == self.robot_id:
            return  # Ignore our own requests
        
        with self.lock:
            if self.current_collab_task and self.current_collab_task.id == msg.task_id:
                # Calculate our leadership score for comparison
                distance_factor = 1.0  # Placeholder
                workload_factor = 1.0 / (1.0 + self.workload)
                capability_factor = 0.8  # Placeholder
                
                our_score = (
                    self.leadership_weights[0] * distance_factor +
                    self.leadership_weights[1] * workload_factor +
                    self.leadership_weights[2] * capability_factor
                )
                
                # Determine leader based on scores
                self.is_leader = our_score > msg.leadership_score
                
                # Send response
                response = CollaborationResponse()
                response.task_id = msg.task_id
                response.robot_id = self.robot_id
                response.agree_to_role = True
                response.is_leader = self.is_leader
                response.timestamp = int(time.time() * 1000)
                
                self.collab_response_pub.publish(response)
                
                role = "Leader" if self.is_leader else "Follower"
                self.get_logger().info(
                    f'Determined role for task {msg.task_id}: {role}')
    
    def collab_response_callback(self, msg):
        """Process collaboration response to finalize roles."""
        if msg.robot_id == self.robot_id:
            return  # Ignore our own responses
        
        with self.lock:
            if self.current_collab_task and self.current_collab_task.id == msg.task_id:
                # Update role based on response
                self.is_leader = not msg.is_leader
                
                role = "Leader" if self.is_leader else "Follower"
                self.get_logger().info(
                    f'Finalized role for task {msg.task_id}: {role}')
    
    def execution_step(self):
        """Execute collaborative task with synchronization."""
        with self.lock:
            if not self.current_collab_task:
                return
            
            task_id = self.current_collab_task.id
            
            # Handle synchronization based on role
            if self.is_leader:
                self.leader_execution_step(task_id)
            else:
                self.follower_execution_step(task_id)
    
    def leader_execution_step(self, task_id):
        """Leader's execution logic with synchronization."""
        current_state = self.sync_states.get(task_id, 0)
        sync_points = self.sync_points.get(task_id, [])
        
        if current_state >= len(sync_points):
            # Task complete
            self.complete_collaborative_task(task_id)
            return
        
        # Check if we're at a synchronization point
        if self.current_sync_point < len(sync_points):
            target_progress = sync_points[self.current_sync_point]
            
            # Simulate progress (in real implementation, would track actual execution)
            current_progress = current_state / max(1, len(sync_points))
            
            if current_progress >= target_progress:
                # Send synchronization signal
                self.send_sync_signal(task_id, self.current_sync_point, "READY")
                
                # Wait for follower acknowledgment
                current_time = time.time()
                if current_time - self.last_sync_time > self.sync_timeout:
                    # Timeout waiting for follower
                    self.get_logger().warn(
                        f'Sync timeout for task {task_id} at point {self.current_sync_point}')
                    
                    # Resend sync signal
                    self.send_sync_signal(task_id, self.current_sync_point, "READY")
                    self.last_sync_time = current_time
            else:
                # Continue execution until next sync point
                self.sync_states[task_id] = current_state + 0.01  # Simulated progress
    
    def follower_execution_step(self, task_id):
        """Follower's execution logic with synchronization."""
        current_state = self.sync_states.get(task_id, 0)
        sync_points = self.sync_points.get(task_id, [])
        
        if current_state >= len(sync_points):
            # Task complete
            self.complete_collaborative_task(task_id)
            return
        
        # Continue execution until leader signals
        if self.current_sync_point < len(sync_points):
            target_progress = sync_points[self.current_sync_point]
            
            # Simulate progress
            current_progress = current_state / max(1, len(sync_points))
            
            if current_progress < target_progress:
                # Continue execution until sync point
                self.sync_states[task_id] = current_state + 0.01  # Simulated progress
    
    def send_sync_signal(self, task_id, sync_point, status):
        """Send synchronization signal."""
        signal = SynchronizationSignal()
        signal.task_id = task_id
        signal.robot_id = self.robot_id
        signal.sync_point = sync_point
        signal.status = status
        signal.timestamp = int(time.time() * 1000)
        
        self.sync_signal_pub.publish(signal)
        self.last_sync_time = time.time()
        
        self.get_logger().info(
            f'Sent sync signal for task {task_id}, point {sync_point}: {status}')
    
    def sync_signal_callback(self, msg):
        """Process synchronization signals from the other robot."""
        with self.lock:
            if not self.current_collab_task or self.current_collab_task.id != msg.task_id:
                return
            
            task_id = msg.task_id
            
            if self.is_leader and msg.status == "ACKNOWLEDGE":
                # Follower has acknowledged, proceed to next sync point
                self.current_sync_point += 1
                self.get_logger().info(
                    f'Received acknowledgment for task {task_id}, '
                    f'proceeding to sync point {self.current_sync_point}')
                
            elif not self.is_leader and msg.status == "READY":
                # Leader is ready at sync point, send acknowledgment
                self.send_sync_signal(task_id, msg.sync_point, "ACKNOWLEDGE")
                self.current_sync_point = msg.sync_point + 1
                self.get_logger().info(
                    f'Received ready signal for task {task_id}, '
                    f'acknowledged sync point {msg.sync_point}')
    
    def complete_collaborative_task(self, task_id):
        """Mark collaborative task as complete."""
        with self.lock:
            self.get_logger().info(f'Completed collaborative task {task_id}')
            
            # Remove task from active list
            if self.current_collab_task and self.current_collab_task.id == task_id:
                self.current_collab_task = None
                self.is_leader = False
                self.current_sync_point = 0
            
            # Clean up task data
            if task_id in self.sync_states:
                del self.sync_states[task_id]
            
            if task_id in self.sync_points:
                del self.sync_points[task_id]