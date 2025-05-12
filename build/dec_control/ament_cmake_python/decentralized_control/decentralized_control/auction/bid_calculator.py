#!/usr/bin/env python3

import numpy as np

class BidCalculator:
    """
    Utility class for calculating bids in the auction algorithm.
    Implements the bid calculation formulas from the paper.
    """
    
    def __init__(self, alpha, beta):
        """
        Initialize the bid calculator with appropriate weights.
        
        Parameters:
        -----------
        alpha : list
            Weighting parameters for bid calculation [α₁, α₂, α₃, α₄, α₅]
        beta : list
            Weighting parameters for recovery [β₁, β₂]
        """
        self.alpha = alpha
        self.beta = beta
    
    def calculate_bid(self, robot_id, task_id, robot_position, task_position, 
                      robot_capabilities, task_capabilities, robot_workload, 
                      workload_ratio, workload_imbalance, in_recovery=False,
                      unassigned_iter=0, utility_iter=0, task_oscillation_count=0):
        """
        Calculate a bid for a task based on multiple factors.
        
        Parameters:
        -----------
        robot_id : int
            ID of the robot making the bid
        task_id : int
            ID of the task being bid on
        robot_position : numpy.ndarray
            3D position of the robot
        task_position : numpy.ndarray
            3D position of the task
        robot_capabilities : numpy.ndarray
            Capability vector of the robot
        task_capabilities : numpy.ndarray
            Required capabilities for the task
        robot_workload : float
            Current workload of the robot
        workload_ratio : float
            Ratio of robot's workload to maximum workload
        workload_imbalance : float
            Measure of imbalance between robot workloads
        in_recovery : bool
            Whether in recovery mode after failure
        unassigned_iter : int
            How many iterations the task has been unassigned
        utility_iter : int
            Current iteration number
        task_oscillation_count : int
            How many times the task has changed robots
            
        Returns:
        --------
        bid : float
            Calculated bid value
        """
        # Calculate distance factor - Euclidean distance in 3D
        distance = np.linalg.norm(robot_position - task_position)
        d_factor = 1 / (1 + distance)
        
        # Configuration cost - simplified for now
        c_factor = 1
        
        # Calculate capability matching - cosine similarity
        if len(robot_capabilities) > 0 and len(task_capabilities) > 0:
            # Normalize vectors to unit length
            robot_cap_norm = robot_capabilities / np.linalg.norm(robot_capabilities)
            task_cap_norm = task_capabilities / np.linalg.norm(task_capabilities)
            
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
        if unassigned_iter > 0:
            # Exponential bonus scaling for persistent unassigned tasks
            if unassigned_iter > 25:
                task_unassigned_bonus = min(6.0, 0.5 * np.exp(unassigned_iter/10))
            elif unassigned_iter > 15:
                task_unassigned_bonus = min(4.0, 0.4 * np.exp(unassigned_iter/12))
            elif unassigned_iter > 5:
                task_unassigned_bonus = min(3.0, 0.3 * unassigned_iter)
        
        # Add scaling factor for tasks that need to be assigned quickly
        iteration_factor = 0
        if utility_iter > 20 and unassigned_iter > 0:
            iteration_factor = min(2.0, 0.05 * utility_iter)
        
        # Calculate bid based on recovery mode
        if in_recovery:
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