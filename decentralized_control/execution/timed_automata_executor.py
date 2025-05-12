# decentralized_control/execution/timed_automata_executor.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time
from threading import Lock
from enum import Enum

from decen_control.msg import Task, TaskList, TaskAssignment, CollaborationRequest
from decen_control.msg import CollaborationResponse, SynchronizationSignal

class State(Enum):
    """States for the timed automaton."""
    INITIAL = 0
    EXECUTION = 1
    SYNCHRONIZATION = 2
    ERROR = 3
    COMPLETED = 4

class TimedAutomataExecutor(Node):
    """
    Implements collaborative task execution using the timed automata formalism
    as described in the dissertation.
    """
    
    def __init__(self):
        super().__init__('timed_automata_executor')
        
        # Get robot ID from parameter
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('invariant_times', [60.0, 30.0, 10.0, 5.0, 60.0])
        self.declare_parameter('sync_timeout', 5.0)
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.invariant_times = self.get_parameter('invariant_times').value
        self.sync_timeout = self.get_parameter('sync_timeout').value
        
        # Follower ID (assuming dual robot system)
        self.follower_id = 3 - self.robot_id  # 1 or 2
        
        # Initialize internal state variables
        self.tasks = {}  # task_id -> Task
        self.collaborative_tasks = {}  # task_id -> Task (collaborative only)
        self.current_task_id = None
        self.current_state = State.INITIAL
        self.is_leader = False
        self.clock = 0.0  # Clock variable for timed automaton
        self.clock_start = 0.0  # Time when clock started
        self.current_sync_point = 0
        self.automaton_states = {}  # task_id -> {state, clock, sync_point}
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
        
        # Timer for automaton execution
        self.automaton_timer = self.create_timer(0.1, self.execute_automaton)
        
        self.get_logger().info(
            f'Timed automata executor initialized for Robot {self.robot_id}')
    
    def task_callback(self, msg):
        """Process incoming tasks and identify collaborative ones."""
        with self.lock:
            for task in msg.tasks:
                self.tasks[task.id] = task
                
                # Identify collaborative tasks
                if task.requires_collaboration:
                    self.collaborative_tasks[task.id] = task
                    
                    # Initialize automaton state for this task
                    self.automaton_states[task.id] = {
                        'state': State.INITIAL,
                        'clock': 0.0,
                        'sync_point': 0,
                        'sync_points': self.generate_sync_points(task)
                    }
                    
                    self.get_logger().info(
                        f'Added collaborative task {task.id} with '
                        f'{len(self.automaton_states[task.id]["sync_points"])} sync points')
    
    def generate_sync_points(self, task):
        """Generate synchronization points for a collaborative task."""
        # In a real implementation, this would depend on task type
        # Here we create evenly spaced sync points
        num_sync_points = 4
        return [i / num_sync_points for i in range(1, num_sync_points + 1)]
    
    def assignment_callback(self, msg):
        """Process task assignments to determine assigned collaborative tasks."""
        with self.lock:
            # Check if any collaborative tasks are assigned to this robot
            for i in range(len(msg.task_ids)):
                task_id = msg.task_ids[i]
                robot_id = msg.robot_ids[i]
                
                # Check if this task is in our collaborative tasks list
                if task_id in self.collaborative_tasks and robot_id == self.robot_id:
                    # We're assigned to this collaborative task
                    if self.current_task_id != task_id:
                        self.current_task_id = task_id
                        
                        # Reset automaton
                        self.automaton_states[task_id] = {
                            'state': State.INITIAL,
                            'clock': 0.0,
                            'sync_point': 0,
                            'sync_points': self.automaton_states[task_id]['sync_points']
                        }
                        
                        # Initiate leader selection
                        self.initiate_leader_selection(task_id)
    
    def initiate_leader_selection(self, task_id):
        """Initiate leader selection for a collaborative task."""
        # Calculate leadership score based on various factors
        leadership_score = np.random.random()  # Simplified for demonstration
        
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
        """Process collaboration requests from other robots."""
        if msg.robot_id == self.robot_id:
            return  # Ignore our own requests
        
        with self.lock:
            if self.current_task_id and self.current_task_id == msg.task_id:
                # Calculate our leadership score for comparison
                our_score = np.random.random()  # Simplified for demonstration
                
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
                
                # Update automaton state
                if self.current_task_id in self.automaton_states:
                    self.automaton_states[self.current_task_id]['state'] = State.EXECUTION
                    self.automaton_states[self.current_task_id]['clock'] = 0.0
                    self.clock_start = time.time()
    
    def collab_response_callback(self, msg):
        """Process collaboration response to finalize roles."""
        if msg.robot_id == self.robot_id:
            return  # Ignore our own responses
        
        with self.lock:
            if self.current_task_id and self.current_task_id == msg.task_id:
                # Update role based on response
                self.is_leader = not msg.is_leader
                
                role = "Leader" if self.is_leader else "Follower"
                self.get_logger().info(
                    f'Finalized role for task {msg.task_id}: {role}')
                
                # Update automaton state
                if self.current_task_id in self.automaton_states:
                    self.automaton_states[self.current_task_id]['state'] = State.EXECUTION
                    self.automaton_states[self.current_task_id]['clock'] = 0.0
                    self.clock_start = time.time()
    
    def execute_automaton(self):
        """Execute the timed automaton for collaborative tasks."""
        with self.lock:
            if not self.current_task_id:
                return
            
            # Get current automaton state
            if self.current_task_id not in self.automaton_states:
                return
            
            automaton = self.automaton_states[self.current_task_id]
            state = automaton['state']
            sync_point = automaton['sync_point']
            sync_points = automaton['sync_points']
            
            # Update clock
            current_time = time.time()
            self.clock = current_time - self.clock_start
            automaton['clock'] = self.clock
            
            # Handle states
            if state == State.EXECUTION:
                # Check if we've reached a synchronization point
                if sync_point < len(sync_points):
                    target_progress = sync_points[sync_point]
                    current_progress = self.clock / self.invariant_times[State.EXECUTION.value]
                    
                    if current_progress >= target_progress:
                        # Transition to SYNCHRONIZATION state
                        automaton['state'] = State.SYNCHRONIZATION
                        self.clock_start = current_time  # Reset clock for new state
                        automaton['clock'] = 0.0
                        
                        if self.is_leader:
                            # Send synchronization signal
                            self.send_sync_signal(self.current_task_id, sync_point, "READY")
                            self.get_logger().info(
                                f'Leader reached sync point {sync_point}, sent READY signal')
                        else:
                            self.get_logger().info(
                                f'Follower reached sync point {sync_point}, waiting for READY signal')
                
                # Check invariant
                if self.clock > self.invariant_times[State.EXECUTION.value]:
                    # Invariant violated, transition to ERROR state
                    automaton['state'] = State.ERROR
                    self.get_logger().error(
                        f'Execution invariant violated for task {self.current_task_id}')
            
            elif state == State.SYNCHRONIZATION:
                # Check invariant
                if self.clock > self.invariant_times[State.SYNCHRONIZATION.value]:
                    # Invariant violated, transition to ERROR state
                    automaton['state'] = State.ERROR
                    self.get_logger().error(
                        f'Synchronization invariant violated for task {self.current_task_id}')
            
            elif state == State.ERROR:
                # Recovery logic
                if self.clock > self.invariant_times[State.ERROR.value]:
                    # Attempt recovery by resetting to EXECUTION state
                    automaton['state'] = State.EXECUTION
                    self.clock_start = current_time
                    automaton['clock'] = 0.0
                    
                    self.get_logger().info(
                        f'Attempting recovery for task {self.current_task_id}')
            
            elif state == State.COMPLETED:
                # Task is completed
                if self.clock > self.invariant_times[State.COMPLETED.value]:
                    # Reset for next task
                    self.current_task_id = None
                    self.get_logger().info('Collaborative task completed')
    
    def sync_signal_callback(self, msg):
        """Process synchronization signals from the other robot."""
        with self.lock:
            if not self.current_task_id or self.current_task_id != msg.task_id:
                return
            
            automaton = self.automaton_states[self.current_task_id]
            
            if automaton['state'] != State.SYNCHRONIZATION:
                return
            
            if self.is_leader and msg.status == "ACKNOWLEDGE":
                # Follower has acknowledged, proceed to next sync point
                automaton['sync_point'] += 1
                
                if automaton['sync_point'] >= len(automaton['sync_points']):
                    # All sync points completed, transition to COMPLETED state
                    automaton['state'] = State.COMPLETED
                    self.clock_start = time.time()
                    automaton['clock'] = 0.0
                    
                    self.get_logger().info(
                        f'All sync points completed for task {self.current_task_id}')
                else:
                    # Continue execution for next sync point
                    automaton['state'] = State.EXECUTION
                    self.clock_start = time.time()
                    automaton['clock'] = 0.0
                    
                    self.get_logger().info(
                        f'Proceeding to sync point {automaton["sync_point"]}')
                
            elif not self.is_leader and msg.status == "READY":
                # Leader is ready at sync point, send acknowledgment
                self.send_sync_signal(self.current_task_id, msg.sync_point, "ACKNOWLEDGE")
                
                # After acknowledgment, move to EXECUTION state for next sync point
                if msg.sync_point >= len(automaton['sync_points']) - 1:
                    # Last sync point, transition to COMPLETED state
                    automaton['state'] = State.COMPLETED
                    self.clock_start = time.time()
                    automaton['clock'] = 0.0
                    
                    self.get_logger().info(
                        f'All sync points completed for task {self.current_task_id}')
                else:
                    # Continue execution for next sync point
                    automaton['sync_point'] = msg.sync_point + 1
                    automaton['state'] = State.EXECUTION
                    self.clock_start = time.time()
                    automaton['clock'] = 0.0
                    
                    self.get_logger().info(
                        f'Acknowledged sync point {msg.sync_point}, '
                        f'proceeding to sync point {automaton["sync_point"]}')
    
    def send_sync_signal(self, task_id, sync_point, status):
        """Send synchronization signal."""
        signal = SynchronizationSignal()
        signal.task_id = task_id
        signal.robot_id = self.robot_id
        signal.sync_point = sync_point
        signal.status = status
        signal.timestamp = int(time.time() * 1000)
        
        self.sync_signal_pub.publish(signal)

def main(args=None):
    rclpy.init(args=args)
    node = TimedAutomataExecutor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()