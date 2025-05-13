#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import time
from threading import Lock

from decentralized_control.auction.bid_calculator import BidCalculator
from dec_control.msg import Task, TaskList, Bid, TaskAssignment, RobotState, Heartbeat
from dec_control.srv import InitAuction, SubmitBid, GetAvailableTasks

class AuctionNode(Node):
    """
    Implements the distributed auction algorithm for task allocation.
    Each robot runs an instance of this node to participate in the auction.
    """
    
    def __init__(self):
        super().__init__('auction_node')
        
        # Add lock for thread safety
        self.lock = Lock()
        
        # Get robot ID from parameter
        self.declare_parameter('robot_id', 1)
        self.robot_id = self.get_parameter('robot_id').value
        
        # Initialize auction parameters
        self.declare_parameter('epsilon', 0.05)
        self.declare_parameter('alpha', [0.8, 0.3, 1.0, 1.2, 0.2])
        self.declare_parameter('gamma', 0.5)
        self.declare_parameter('lambda', 0.1)
        self.declare_parameter('beta', [2.0, 1.5])
        
        # Get parameters
        self.epsilon = self.get_parameter('epsilon').value
        self.alpha = self.get_parameter('alpha').value
        self.gamma = self.get_parameter('gamma').value
        self.lambda_val = self.get_parameter('lambda').value
        self.beta = self.get_parameter('beta').value
        
        # Callback group for services
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize internal state variables
        self.tasks = {}  # Dict of task_id -> Task
        self.prices = {}  # Dict of task_id -> price
        self.assignments = {}  # Dict of task_id -> robot_id
        self.initial_assignments = {}  # Store for recovery analysis
        self.completion_status = {}  # Dict of task_id -> completion status
        self.available_tasks = []  # List of available task IDs
        self.bids = {}  # Dict of (robot_id, task_id) -> bid_value
        self.utilities = {}  # Dict of (robot_id, task_id) -> utility
        self.last_update_time = {}  # Dict of robot_id -> last update time
        self.recovery_mode = False
        self.task_oscillation_count = {}  # Dict of task_id -> oscillation count
        self.task_last_robot = {}  # Dict of task_id -> last robot assigned
        self.unassigned_iterations = {}  # Dict of task_id -> iterations unassigned
        self.utility_iter = 0  # Current iteration for utility history
        
        # Initialize ROS2 communication
        
        # Publishers
        self.bid_publisher = self.create_publisher(
            Bid, f'/robot{self.robot_id}/auction/bids', 10)
        
        self.assignment_publisher = self.create_publisher(
            TaskAssignment, '/auction/assignments', 10)
        
        self.state_publisher = self.create_publisher(
            RobotState, f'/robot{self.robot_id}/state', 10)
        
        self.heartbeat_publisher = self.create_publisher(
            Heartbeat, f'/robot{self.robot_id}/heartbeat', 10)
        
        # Subscribers
        self.task_subscriber = self.create_subscription(
            TaskList, '/tasks', self.task_callback, 10)
        
        self.bid_subscribers = []
        for i in range(1, 3):  # For dual mobile manipulators
            if i != self.robot_id:
                self.bid_subscribers.append(
                    self.create_subscription(
                        Bid, f'/robot{i}/auction/bids', self.bid_callback, 10))
        
        self.assignment_subscriber = self.create_subscription(
            TaskAssignment, '/auction/assignments', self.assignment_callback, 10)
        
        self.robot_state_subscribers = []
        for i in range(1, 3):  # For dual mobile manipulators
            if i != self.robot_id:
                self.robot_state_subscribers.append(
                    self.create_subscription(
                        RobotState, f'/robot{i}/state', self.robot_state_callback, 10))
        
        self.heartbeat_subscribers = []
        for i in range(1, 3):  # For dual mobile manipulators
            if i != self.robot_id:
                self.heartbeat_subscribers.append(
                    self.create_subscription(
                        Heartbeat, f'/robot{i}/heartbeat', self.heartbeat_callback, 10))
        
        # Services
        self.init_auction_service = self.create_service(
            InitAuction, f'/robot{self.robot_id}/auction/init', 
            self.init_auction_callback, callback_group=self.callback_group)
        
        self.submit_bid_service = self.create_service(
            SubmitBid, f'/robot{self.robot_id}/auction/submit_bid', 
            self.submit_bid_callback, callback_group=self.callback_group)
        
        self.get_available_tasks_service = self.create_service(
            GetAvailableTasks, f'/robot{self.robot_id}/auction/get_available_tasks', 
            self.get_available_tasks_callback, callback_group=self.callback_group)
        
        # Timers
        self.auction_timer = self.create_timer(
            0.1, self.auction_step_callback)  # 10Hz auction update
        
        self.heartbeat_timer = self.create_timer(
            1.0, self.send_heartbeat)  # 1Hz heartbeat
        
        self.publish_state_timer = self.create_timer(
            0.2, self.publish_state)  # 5Hz state publishing
        
        # Initialize bid calculator
        self.bid_calculator = BidCalculator(self.alpha, self.beta)
        
        # Robot state
        self.position = np.zeros(3)
        self.orientation = np.zeros(3)
        self.capabilities = np.ones(5)
        self.workload = 0.0
        self.failed = False
        
        # Log initialization
        self.get_logger().info(
            f'Auction node initialized for Robot {self.robot_id} with epsilon={self.epsilon}')
    
    def task_callback(self, msg):
        """Process incoming tasks"""
        with self.lock:
            for task in msg.tasks:
                self.tasks[task.id] = task
                
                # Initialize task state if new
                if task.id not in self.prices:
                    self.prices[task.id] = 0.0
                    self.assignments[task.id] = 0  # Unassigned
                    self.initial_assignments[task.id] = 0
                    self.completion_status[task.id] = 0  # Not completed
                    self.task_oscillation_count[task.id] = 0
                    self.task_last_robot[task.id] = 0
                    self.unassigned_iterations[task.id] = 0
                
                # Update available tasks
                self.update_available_tasks()
    
    def bid_callback(self, msg):
        """Process incoming bids from other robots"""
        if self.failed:
            return
        
        robot_id = msg.robot_id
        task_id = msg.task_id
        bid_value = msg.bid_value
        
        # Store the bid
        self.bids[(robot_id, task_id)] = bid_value
        
        # Update utility
        self.utilities[(robot_id, task_id)] = bid_value - self.prices.get(task_id, 0.0)
        
        # Process bid (make sure task_id exists in self.prices before processing)
        if task_id in self.prices:
            self.process_bid(robot_id, task_id, bid_value)
        
    def assignment_callback(self, msg):
        """Process task assignment updates"""
        if self.failed:
            return
            
        with self.lock:
            # Update assignments from consensus
            for i in range(len(msg.task_ids)):
                task_id = msg.task_ids[i]
                robot_id = msg.robot_ids[i]
                price = msg.prices[i]
                
                # Track oscillations
                if task_id in self.assignments and self.assignments[task_id] != robot_id and self.assignments[task_id] > 0:
                    self.task_oscillation_count[task_id] += 1
                    self.task_last_robot[task_id] = robot_id
                
                # Update local state
                self.assignments[task_id] = robot_id
                self.prices[task_id] = price
                
                # If this is the first assignment, record for recovery analysis
                if self.initial_assignments[task_id] == 0 and robot_id > 0:
                    self.initial_assignments[task_id] = robot_id
    
    def robot_state_callback(self, msg):
        """Process state information from other robots"""
        self.last_update_time[msg.id] = time.time()
        
        # Record if a robot has failed
        if msg.failed and not self.recovery_mode:
            self.get_logger().info(f'Detected Robot {msg.id} failure, initiating recovery')
            self.initiate_recovery(msg.id)
    
    def heartbeat_callback(self, msg):
        """Process heartbeat signals for failure detection"""
        self.last_update_time[msg.robot_id] = time.time()
    
    def init_auction_callback(self, request, response):
        """Initialize the auction process"""
        self.get_logger().info('Initializing auction')
        
        # Reset auction state
        self.prices = {task_id: 0.0 for task_id in self.tasks}
        self.assignments = {task_id: 0 for task_id in self.tasks}
        self.initial_assignments = {task_id: 0 for task_id in self.tasks}
        self.completion_status = {task_id: 0 for task_id in self.tasks}
        self.task_oscillation_count = {task_id: 0 for task_id in self.tasks}
        self.task_last_robot = {task_id: 0 for task_id in self.tasks}
        self.unassigned_iterations = {task_id: 0 for task_id in self.tasks}
        self.utility_iter = 0
        self.recovery_mode = False
        
        response.success = True
        return response
    
    def submit_bid_callback(self, request, response):
        """Service to manually submit a bid"""
        if self.failed:
            response.success = False
            return response
        
        task_id = request.task_id
        bid_value = request.bid_value
        
        # Create and publish bid
        bid_msg = Bid()
        bid_msg.robot_id = self.robot_id
        bid_msg.task_id = task_id
        bid_msg.bid_value = bid_value
        bid_msg.utility = bid_value - self.prices[task_id]
        
        self.bid_publisher.publish(bid_msg)
        
        # Update local state
        self.bids[(self.robot_id, task_id)] = bid_value
        self.utilities[(self.robot_id, task_id)] = bid_value - self.prices[task_id]
        
        # Process the bid locally
        self.process_bid(self.robot_id, task_id, bid_value)
        
        response.success = True
        return response
    
    def get_available_tasks_callback(self, request, response):
        """Service to get currently available tasks"""
        response.task_ids = self.available_tasks
        return response
    
    def auction_step_callback(self):
        """
        Main auction algorithm step.
        This function is called periodically to execute the distributed auction algorithm.
        """
        if self.failed or not self.tasks:
            return
        
        with self.lock:
            # Update available tasks
            self.update_available_tasks()
            
            # Skip if no available tasks
            if not self.available_tasks:
                return
            
            # Update unassigned iterations counter
            for task_id in self.tasks:
                if self.assignments[task_id] == 0:
                    self.unassigned_iterations[task_id] += 1
                else:
                    self.unassigned_iterations[task_id] = 0
            
            # Mark "in recovery" tasks as unassigned before bidding
            recovery_tasks = [task_id for task_id, robot_id in self.assignments.items() if robot_id == -1]
            for task_id in recovery_tasks:
                self.assignments[task_id] = 0
                if task_id not in self.available_tasks:
                    self.available_tasks.append(task_id)
            
            # Calculate robot workloads
            robot_workloads = self.calculate_robot_workloads()
            
            # Adaptive epsilon based on iteration
            if self.utility_iter < 10:
                base_epsilon = self.epsilon * 1.5  # Higher initial epsilon to prevent oscillations
            else:
                base_epsilon = self.epsilon
            
            # Calculate workload ratio and imbalance
            workload_ratio = 1.0
            workload_imbalance = 0.0
            max_workload = max(robot_workloads.values()) if robot_workloads else 0
            
            if max_workload > 0:
                workload_ratio = robot_workloads.get(self.robot_id, 0) / max_workload
                min_workload = min(robot_workloads.values()) if robot_workloads else 0
                workload_diff = max_workload - min_workload
                workload_imbalance = workload_diff / max_workload if max_workload > 0 else 0
            
            # Adaptive batch sizing
            if self.utility_iter < 10:
                base_batch_size = np.ceil(len(self.available_tasks)/3)  # More aggressive initially
            else:
                base_batch_size = np.ceil(len(self.available_tasks)/5)  # Base batch size
            
            # Adjust batch size based on workload imbalance
            if workload_imbalance > 0.3:
                imbalance_factor = 1.5  # Larger batches when imbalance is high
            elif workload_imbalance > 0.15:
                imbalance_factor = 1.25
            else:
                imbalance_factor = 1.0
            
            # Adjust batch size based on unassigned tasks
            unassigned_count = sum(1 for r in self.assignments.values() if r == 0)
            if unassigned_count > 0:
                unassigned_factor = 1 + (unassigned_count / len(self.tasks)) * 0.5
            else:
                unassigned_factor = 1.0
            
            # Calculate final batch size
            max_bids_per_iteration = max(2, int(np.ceil(base_batch_size * imbalance_factor * unassigned_factor)))
            max_bids_per_iteration = min(max_bids_per_iteration, len(self.available_tasks))
            
            # Calculate bids for available tasks
            task_utilities = {}
            for task_id in self.available_tasks:
                # Skip if already assigned to this robot
                if self.assignments[task_id] == self.robot_id:
                    continue
                    
                # Calculate bid with enhanced global objective consideration
                bid_value = self.calculate_bid(
                    task_id, robot_workloads.get(self.robot_id, 0), 
                    workload_ratio, workload_imbalance)
                    
                self.bids[(self.robot_id, task_id)] = bid_value
                utility = bid_value - self.prices[task_id]
                
                # Apply penalty if this task has recently oscillated between robots
                if self.task_oscillation_count.get(task_id, 0) > 3 and self.task_last_robot.get(task_id, 0) != self.robot_id:
                    utility *= 0.9
                    
                # Special handling for long-unassigned tasks with escalating incentives
                if self.assignments[task_id] == 0:
                    unassigned_iter = self.unassigned_iterations.get(task_id, 0)
                    if unassigned_iter > 20:
                        bonus_factor = min(3.0, 1.0 + (unassigned_iter - 20) * 0.1)
                        utility *= bonus_factor
                        
                self.utilities[(self.robot_id, task_id)] = utility
                task_utilities[task_id] = utility
            
            # Sort tasks by utility
            sorted_tasks = sorted(
                task_utilities.items(), key=lambda x: x[1], reverse=True)
            
            # Select best tasks with positive utility - limited by batch size
            bid_count = 0
            for task_id, utility in sorted_tasks:
                if utility > 0 and bid_count < max_bids_per_iteration:
                    # Create and publish bid
                    bid_msg = Bid()
                    bid_msg.robot_id = self.robot_id
                    bid_msg.task_id = task_id
                    bid_msg.bid_value = self.bids[(self.robot_id, task_id)]
                    bid_msg.utility = utility
                    
                    self.bid_publisher.publish(bid_msg)
                    
                    # Process bid locally as well
                    old_assignment = self.assignments[task_id]
                    
                    # Track oscillation (robot changes) for this task
                    if old_assignment > 0 and old_assignment != self.robot_id:
                        self.task_oscillation_count[task_id] = self.task_oscillation_count.get(task_id, 0) + 1
                    self.task_last_robot[task_id] = self.robot_id
                    
                    # Update assignment and price
                    self.assignments[task_id] = self.robot_id
                    
                    # Dynamic price increment with multiple factors
                    effective_epsilon = base_epsilon
                    
                    # Factor 1: Task oscillation history
                    oscillation_count = self.task_oscillation_count.get(task_id, 0)
                    if oscillation_count > 2:
                        effective_epsilon *= (1 + 0.15 * oscillation_count)
                    
                    # Factor 2: Workload balancing
                    if workload_ratio > 1.2:  # Robot has >20% more workload than minimum
                        effective_epsilon *= 1.5  # Increase price faster
                    elif workload_ratio < 0.8:  # Robot has <80% of maximum workload
                        effective_epsilon *= 0.7  # Increase price slower
                    
                    # Cap prices to prevent them from getting too high
                    max_price = 3.0 * max(self.alpha)  # Maximum reasonable price
                    if self.prices[task_id] + effective_epsilon > max_price:
                        effective_epsilon = max(0, max_price - self.prices[task_id])
                    
                    # Update price
                    self.prices[task_id] += effective_epsilon
                    
                    # If this is the first assignment, record it for recovery analysis
                    if self.initial_assignments[task_id] == 0:
                        self.initial_assignments[task_id] = self.robot_id
                    
                    # Update workload in our records
                    if old_assignment > 0:
                        # Reduce workload for previous robot (handled in other robot's node)
                        pass
                    
                    # Update our workload
                    if hasattr(self.tasks[task_id], 'execution_time'):
                        self.workload += self.tasks[task_id].execution_time
                    else:
                        self.workload += 1.0
                    
                    # Publish updated assignments
                    self.publish_assignments()
                    
                    bid_count += 1
            
            # Progressive price reduction for unassigned tasks
            if self.utility_iter > 10:
                unassigned_tasks = [task_id for task_id, robot_id in self.assignments.items() if robot_id == 0]
                
                for task_id in unassigned_tasks:
                    unassigned_iter = self.unassigned_iterations.get(task_id, 0)
                    
                    # Progressive price reduction - more aggressive for longer unassigned tasks
                    if unassigned_iter > 30:
                        reduction_factor = 0.3  # Very aggressive reduction
                    elif unassigned_iter > 20:
                        reduction_factor = 0.5  # Strong reduction
                    elif unassigned_iter > 10:
                        reduction_factor = 0.7  # Moderate reduction
                    else:
                        reduction_factor = 0.9  # Mild reduction
                    
                    self.prices[task_id] *= reduction_factor
                    
                    # Ensure price doesn't go below zero
                    self.prices[task_id] = max(0, self.prices[task_id])
                    
                    # Clear oscillation history to allow fresh bidding
                    if unassigned_iter > 15:
                        self.task_oscillation_count[task_id] = 0
                    
                    # Log status updates for difficult tasks
                    if unassigned_iter > 25 and self.utility_iter % 5 == 0:
                        self.get_logger().info(
                            f'Task {task_id} remains unassigned for {unassigned_iter} iterations - '
                            f'price reduced to {self.prices[task_id]:.2f}')
            
            # Update utility iteration counter
            self.utility_iter += 1
    
    def calculate_bid(self, task_id, robot_workload, workload_ratio, workload_imbalance):
        """Calculate bid for a given task"""
        # Extract task details
        task = self.tasks[task_id]
        
        # Calculate distance factor - Euclidean distance in 3D
        task_position = np.array(task.position)
        distance = np.linalg.norm(self.position - task_position)
        
        # Better distance factor normalization
        d_factor = 1 / (1 + distance)
        
        # Configuration cost - keep simple for now
        c_factor = 1
        
        # Calculate capability matching
        if hasattr(task, 'capabilities_required') and len(task.capabilities_required) > 0:
            robot_cap = np.array(self.capabilities)
            task_cap = np.array(task.capabilities_required)
            
            # Normalize vectors to unit length
            robot_cap_norm = robot_cap / np.linalg.norm(robot_cap)
            task_cap_norm = task_cap / np.linalg.norm(task_cap)
            
            # Compute cosine similarity (normalized dot product)
            capability_match = np.dot(robot_cap_norm, task_cap_norm)
        else:
            capability_match = 0.8  # Default value
        
        # Progressive workload factor with stronger imbalance penalty
        workload_factor = robot_workload / 10 * (workload_ratio**1.8)
        
        # Add global balance consideration
        global_balance_factor = workload_imbalance * 0.5
        
        # Simple energy factor
        energy_factor = distance * 0.1
        
        # Adjust workload penalty when imbalance is high
        adjusted_workload_alpha = self.alpha[3]
        if workload_ratio > 1.2 or workload_ratio < 0.8:
            adjusted_workload_alpha *= 1.5  # 50% increase in penalty
        
        # Enhanced bonus for unassigned tasks with progressive scaling
        task_unassigned_bonus = 0
        if self.assignments[task_id] == 0:
            unassigned_iter = self.unassigned_iterations.get(task_id, 0)
            
            # Exponential bonus scaling for persistent unassigned tasks
            if unassigned_iter > 25:
                task_unassigned_bonus = min(6.0, 0.5 * np.exp(unassigned_iter/10))
            elif unassigned_iter > 15:
                task_unassigned_bonus = min(4.0, 0.4 * np.exp(unassigned_iter/12))
            elif unassigned_iter > 5:
                task_unassigned_bonus = min(3.0, 0.3 * unassigned_iter)
        
        # Add scaling factor for tasks that need to be assigned quickly
        iteration_factor = 0
        if self.utility_iter > 20 and self.assignments[task_id] == 0:
            iteration_factor = min(2.0, 0.05 * self.utility_iter)
        
        # Calculate bid based on recovery mode
        if self.recovery_mode:
            # Add recovery-specific terms when in recovery mode
            # Calculate bid components with improved recovery bias
            progress_term = self.beta[0] * (1 - 0)  # Assuming no partial progress
            criticality_term = self.beta[1] * 0.5   # Default criticality
            
            # Recovery bids favor robots with lower workload more strongly
            recovery_workload_factor = (1 - workload_ratio) * self.beta[0] * 0.8
            
            # Add stronger global balance factor during recovery
            global_recovery_factor = workload_imbalance * (self.beta[1] if len(self.beta) >= 2 else 1.0) * 0.7
            
            bid = (self.alpha[0] * d_factor + 
                   self.alpha[1] * c_factor + 
                   self.alpha[2] * capability_match - 
                   adjusted_workload_alpha * workload_factor - 
                   self.alpha[4] * energy_factor + 
                   progress_term + 
                   criticality_term + 
                   recovery_workload_factor + 
                   global_recovery_factor + 
                   global_balance_factor + 
                   task_unassigned_bonus + 
                   iteration_factor)
        else:
            bid = (self.alpha[0] * d_factor + 
                   self.alpha[1] * c_factor + 
                   self.alpha[2] * capability_match - 
                   adjusted_workload_alpha * workload_factor - 
                   self.alpha[4] * energy_factor + 
                   global_balance_factor + 
                   task_unassigned_bonus + 
                   iteration_factor)
        
        # Add small random noise to break ties
        bid += 0.001 * np.random.random()
        
        return bid
    
    def process_bid(self, robot_id, task_id, bid_value):
        """Process a bid from any robot (including self)"""
        if task_id not in self.prices:
            return  # Task doesn't exist in our database
        
        old_assignment = self.assignments.get(task_id, 0)
        
        # Calculate utility
        utility = bid_value - self.prices[task_id]
        
        if utility > 0:
            # Track oscillation (robot changes) for this task
            if old_assignment > 0 and old_assignment != robot_id:
                self.task_oscillation_count[task_id] = self.task_oscillation_count.get(task_id, 0) + 1
            self.task_last_robot[task_id] = robot_id
            
            # Update assignment and price
            self.assignments[task_id] = robot_id
            self.prices[task_id] += self.epsilon
            
            # If this is the first assignment, record it for recovery analysis
            if task_id not in self.initial_assignments or self.initial_assignments[task_id] == 0:
                self.initial_assignments[task_id] = robot_id
            
            # Publish updated assignments
            self.publish_assignments()
    
    def update_available_tasks(self):
        """Update the list of available tasks based on prerequisites"""
        completed_tasks = [task_id for task_id, status in self.completion_status.items() if status == 1]
        
        # Find tasks with all prerequisites completed
        self.available_tasks = []
        for task_id, task in self.tasks.items():
            if self.completion_status[task_id] == 1:
                continue  # Skip completed tasks
            
            # Check prerequisites
            if not hasattr(task, 'prerequisites') or not task.prerequisites:
                self.available_tasks.append(task_id)
            else:
                # Check if all prerequisites are completed
                if all(prereq in completed_tasks for prereq in task.prerequisites):
                    self.available_tasks.append(task_id)
    
    def calculate_robot_workloads(self):
        """Calculate workload for each robot based on assignments"""
        robot_workloads = {1: 0.0, 2: 0.0}  # For dual mobile manipulators
        
        for task_id, robot_id in self.assignments.items():
            if robot_id > 0:  # Skip unassigned tasks
                if hasattr(self.tasks[task_id], 'execution_time'):
                    robot_workloads[robot_id] += self.tasks[task_id].execution_time
                else:
                    robot_workloads[robot_id] += 1.0  # Default value if no execution time
        
        return robot_workloads
    
    def initiate_recovery(self, failed_robot_id):
        """Initiate recovery process after a robot failure"""
        self.recovery_mode = True
        
        # Store current assignment state for recovery analysis
        self.failure_assignment = self.assignments.copy()
        
        # Find tasks assigned to the failed robot
        failed_tasks = [task_id for task_id, robot_id in self.assignments.items() 
                       if robot_id == failed_robot_id]
        
        # Prioritize critical tasks during recovery
        if failed_tasks:
            # Calculate criticality scores for failed tasks
            criticality_scores = {}
            
            for task_id in failed_tasks:
                task = self.tasks[task_id]
                
                # Consider execution time
                if hasattr(task, 'execution_time'):
                    criticality_scores[task_id] = task.execution_time
                else:
                    criticality_scores[task_id] = 1.0
                
                # Add bonus for tasks with dependencies
                if hasattr(task, 'prerequisites') and task.prerequisites:
                    # Count how many tasks depend on this one
                    dependent_count = 0
                    for task_id2, task2 in self.tasks.items():
                        if hasattr(task2, 'prerequisites') and task_id in task2.prerequisites:
                            dependent_count += 1
                    criticality_scores[task_id] += dependent_count * 2
            
            # Sort failed tasks by criticality (highest first)
            prioritized_failed_tasks = sorted(
                criticality_scores.items(), key=lambda x: x[1], reverse=True)
            
            # Process in order of criticality
            for i, (task_id, criticality) in enumerate(prioritized_failed_tasks):
                # Reset oscillation count to avoid bias
                self.task_oscillation_count[task_id] = 0
                
                # Mark as in recovery with priority level reflected in the price reset
                # Most critical tasks get bigger price reductions
                priority_factor = 1 - (i / len(prioritized_failed_tasks))
                self.assignments[task_id] = -1  # -1 indicates "in recovery"
                
                # More aggressive price reset for high-priority tasks
                reset_factor = 0.3 * (1 + priority_factor)
                self.prices[task_id] *= reset_factor
        
        self.get_logger().info(
            f'Recovery initiated for robot {failed_robot_id}. '
            f'{len(failed_tasks)} tasks need reassignment.')
    
    def publish_assignments(self):
        """Publish current task assignments"""
        msg = TaskAssignment()
        
        for task_id, robot_id in self.assignments.items():
            msg.task_ids.append(task_id)
            msg.robot_ids.append(robot_id)
            msg.prices.append(self.prices[task_id])
        
        self.assignment_publisher.publish(msg)
    
    def send_heartbeat(self):
        """Send heartbeat signal"""
        if self.failed:
            return
            
        msg = Heartbeat()
        msg.robot_id = self.robot_id
        msg.timestamp = int(time.time() * 1000)  # milliseconds
        msg.status = 0  # 0 = Normal
        
        self.heartbeat_publisher.publish(msg)
    
    def publish_state(self):
        """Publish robot state information"""
        msg = RobotState()
        msg.id = self.robot_id
        msg.position = self.position.tolist()
        msg.orientation = self.orientation.tolist()
        msg.capabilities = self.capabilities.tolist()
        msg.workload = self.workload
        msg.failed = self.failed
        
        self.state_publisher.publish(msg)