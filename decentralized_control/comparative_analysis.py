# comparative_analysis.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time
from enum import Enum
import json
import os

from dec_control.msg import Task, TaskList, TaskAssignment

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

#!/usr/bin/env python3

import numpy as np
import pandas as pd
from scipy import stats
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import yaml
import json

class ConsensusComparativeAnalysis:
    """
    Performs comparative analysis of different consensus approaches
    across varying network conditions.
    """
    
    def __init__(self, results_dir):
        self.results_dir = Path(results_dir)
        self.load_experiment_config()
        
        # Set up visualization style
        sns.set_style("whitegrid")
        plt.rcParams.update({'font.size': 12})
    
    def load_experiment_config(self):
        """Load experiment configuration."""
        with open('config/experiments.yaml', 'r') as f:
            self.config = yaml.safe_load(f)
    
    def load_results(self):
        """Load all results from experiment runs."""
        results = []
        for result_file in self.results_dir.glob('consensus_results_*.json'):
            with open(result_file, 'r') as f:
                data = json.load(f)
                results.append(data)
        return results
    
    def create_analysis_dataframe(self, results):
        """Convert results to pandas DataFrame for analysis."""
        data = []
        for result_set in results:
            for condition, condition_data in result_set.items():
                for run in condition_data['individual_runs']:
                    row = {
                        'condition': condition,
                        'convergence_time': run['convergence_time'],
                        'total_messages': run['total_messages'],
                        'delay_mean_ms': run['network_conditions']['delay_mean_ms'],
                        'delay_stddev_ms': run['network_conditions']['delay_stddev_ms'],
                        'packet_loss_prob': run['network_conditions']['packet_loss_prob']
                    }
                    # Add message counts by type
                    for msg_type, count in run['message_counts'].items():
                        row[f'messages_{msg_type}'] = count
                    data.append(row)
        
        return pd.DataFrame(data)
    
    def calculate_performance_metrics(self, df):
        """Calculate key performance metrics for each condition."""
        metrics = {}
        for condition in df['condition'].unique():
            condition_data = df[df['condition'] == condition]
            metrics[condition] = {
                'convergence_time': {
                    'mean': condition_data['convergence_time'].mean(),
                    'std': condition_data['convergence_time'].std(),
                    'ci': stats.t.interval(
                        0.95,
                        len(condition_data) - 1,
                        loc=condition_data['convergence_time'].mean(),
                        scale=stats.sem(condition_data['convergence_time'])
                    )
                },
                'message_efficiency': {
                    'mean': condition_data['convergence_time'].mean() / 
                           condition_data['total_messages'].mean(),
                    'std': np.std(condition_data['convergence_time'] / 
                                condition_data['total_messages'])
                },
                'reliability': {
                    'convergence_rate': len(condition_data[condition_data['convergence_time'].notna()]) /
                                      len(condition_data)
                }
            }
        return metrics
    
    def perform_statistical_tests(self, df):
        """Perform statistical analysis comparing algorithms."""
        tests = {}
        
        # ANOVA test for convergence times across conditions
        conditions = df['condition'].unique()
        conv_times_by_condition = [
            df[df['condition'] == cond]['convergence_time'].dropna()
            for cond in conditions
        ]
        f_stat, p_val = stats.f_oneway(*conv_times_by_condition)
        
        tests['anova'] = {
            'statistic': f_stat,
            'p_value': p_val
        }
        
        # Correlation analysis
        corr_tests = {}
        for var in ['delay_mean_ms', 'packet_loss_prob']:
            corr, p = stats.pearsonr(
                df[var],
                df['convergence_time'].fillna(df['convergence_time'].max())
            )
            corr_tests[var] = {
                'correlation': corr,
                'p_value': p
            }
        tests['correlations'] = corr_tests
        
        return tests
    
    def plot_convergence_comparison(self, df, output_file=None):
        """Plot convergence time comparison across conditions."""
        plt.figure(figsize=(12, 6))
        
        # Box plot with individual points
        sns.boxplot(x='condition', y='convergence_time', data=df)
        sns.swarmplot(x='condition', y='convergence_time', data=df, 
                     color='0.25', size=4, alpha=0.5)
        
        plt.title('Consensus Convergence Time by Network Condition')
        plt.xlabel('Network Condition')
        plt.ylabel('Convergence Time (s)')
        plt.xticks(rotation=45)
        
        if output_file:
            plt.savefig(output_file, bbox_inches='tight')
        plt.close()
    
    def plot_efficiency_metrics(self, df, output_file=None):
        """Plot efficiency metrics across conditions."""
        plt.figure(figsize=(15, 5))
        
        plt.subplot(1, 3, 1)
        sns.boxplot(x='condition', y='total_messages', data=df)
        plt.title('Total Messages')
        plt.xticks(rotation=45)
        
        plt.subplot(1, 3, 2)
        efficiency = df['convergence_time'] / df['total_messages']
        sns.boxplot(x='condition', y=efficiency, data=df)
        plt.title('Message Efficiency\n(Time per Message)')
        plt.xticks(rotation=45)
        
        plt.subplot(1, 3, 3)
        reliability = df.groupby('condition')['convergence_time'].apply(
            lambda x: x.notna().mean()
        ).reset_index()
        sns.barplot(x='condition', y='convergence_time', data=reliability)
        plt.title('Convergence Reliability')
        plt.ylabel('Convergence Rate')
        plt.xticks(rotation=45)
        
        plt.tight_layout()
        if output_file:
            plt.savefig(output_file, bbox_inches='tight')
        plt.close()
    
    def plot_network_impact(self, df, output_file=None):
        """Plot impact of network conditions on performance."""
        plt.figure(figsize=(15, 5))
        
        plt.subplot(1, 3, 1)
        sns.scatterplot(x='delay_mean_ms', y='convergence_time', 
                       hue='condition', data=df)
        plt.title('Impact of Network Delay')
        plt.xlabel('Mean Delay (ms)')
        plt.ylabel('Convergence Time (s)')
        
        plt.subplot(1, 3, 2)
        sns.scatterplot(x='packet_loss_prob', y='convergence_time',
                       hue='condition', data=df)
        plt.title('Impact of Packet Loss')
        plt.xlabel('Packet Loss Probability')
        plt.ylabel('Convergence Time (s)')
        
        plt.subplot(1, 3, 3)
        sns.scatterplot(x='delay_mean_ms', y='total_messages',
                       hue='condition', data=df)
        plt.title('Message Overhead vs Delay')
        plt.xlabel('Mean Delay (ms)')
        plt.ylabel('Total Messages')
        
        plt.tight_layout()
        if output_file:
            plt.savefig(output_file, bbox_inches='tight')
        plt.close()
    
    def plot_time_varying_analysis(self, df, output_file=None):
        """Analyze and plot performance under time-varying conditions."""
        plt.figure(figsize=(15, 10))
        
        # Plot 1: Network conditions over time
        plt.subplot(2, 2, 1)
        time_varying_data = df[df['condition'] == 'time_varying']
        sns.lineplot(data=time_varying_data, x='timestamps', y='delay_mean_ms',
                    label='Network Delay')
        plt.title('Network Delay Variation Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Delay (ms)')
        
        # Plot 2: Convergence error evolution
        plt.subplot(2, 2, 2)
        for variant in df['algorithm_variant'].unique():
            variant_data = df[df['algorithm_variant'] == variant]
            sns.lineplot(data=variant_data, x='timestamps', y='consensus_error',
                        label=variant)
        plt.title('Consensus Error Evolution')
        plt.xlabel('Time (s)')
        plt.ylabel('Error Magnitude')
        plt.yscale('log')
        
        # Plot 3: Adaptive parameters
        plt.subplot(2, 2, 3)
        network_aware_data = df[df['algorithm_variant'] == 'network_aware']
        sns.lineplot(data=network_aware_data, x='timestamps', y='gamma_value',
                    label='Gamma')
        plt.title('Adaptation Parameter Evolution')
        plt.xlabel('Time (s)')
        plt.ylabel('Parameter Value')
        
        # Plot 4: Message rate adaptation
        plt.subplot(2, 2, 4)
        for variant in df['algorithm_variant'].unique():
            variant_data = df[df['algorithm_variant'] == variant]
            sns.lineplot(data=variant_data, x='timestamps', y='message_rate',
                        label=variant)
        plt.title('Message Rate Adaptation')
        plt.xlabel('Time (s)')
        plt.ylabel('Messages/s')
        
        plt.tight_layout()
        if output_file:
            plt.savefig(output_file, bbox_inches='tight')
        plt.close()
    
    def analyze_time_varying_performance(self, df):
        """Analyze performance metrics under time-varying conditions."""
        time_varying_metrics = {
            'delay_tracking': {},      # How well algorithms track network changes
            'stability': {},          # Stability of consensus under variations
            'adaptation_speed': {},   # Speed of parameter adaptation
            'efficiency': {}          # Message efficiency over time
        }
        
        for variant in df['algorithm_variant'].unique():
            variant_data = df[df['algorithm_variant'] == variant]
            
            # Calculate delay tracking correlation
            delay_correlation = np.corrcoef(
                variant_data['delay_mean_ms'],
                variant_data['consensus_error']
            )[0,1]
            
            # Calculate stability metrics
            error_stability = np.std(variant_data['consensus_error'])
            param_stability = np.std(variant_data['gamma_value']) if 'gamma_value' in variant_data else None
            
            # Calculate adaptation metrics
            if 'gamma_value' in variant_data:
                delay_changes = np.diff(variant_data['delay_mean_ms'])
                gamma_changes = np.diff(variant_data['gamma_value'])
                adaptation_lag = np.correlate(delay_changes, gamma_changes, mode='full')
                max_lag_idx = np.argmax(np.abs(adaptation_lag))
                adaptation_speed = max_lag_idx - len(delay_changes) + 1
            else:
                adaptation_speed = None
            
            # Calculate message efficiency
            avg_msg_rate = np.mean(variant_data['message_rate'])
            efficiency = 1.0 / (variant_data['consensus_error'].mean() * avg_msg_rate)
            
            time_varying_metrics['delay_tracking'][variant] = delay_correlation
            time_varying_metrics['stability'][variant] = {
                'error_std': error_stability,
                'param_std': param_stability
            }
            time_varying_metrics['adaptation_speed'][variant] = adaptation_speed
            time_varying_metrics['efficiency'][variant] = efficiency
        
        return time_varying_metrics
    
    def generate_time_varying_report(self, metrics, output_file=None):
        """Generate report focusing on time-varying performance."""
        report = []
        report.append("Time-Varying Network Performance Analysis")
        report.append("======================================\n")
        
        # Network delay tracking
        report.append("Network Delay Tracking")
        report.append("---------------------")
        for variant, correlation in metrics['delay_tracking'].items():
            report.append(f"{variant}:")
            report.append(f"  Delay-Error Correlation: {correlation:.4f}")
            report.append(f"  Interpretation: {'Strong' if abs(correlation) > 0.7 else 'Moderate' if abs(correlation) > 0.3 else 'Weak'} "
                        f"{'positive' if correlation > 0 else 'negative'} correlation\n")
        
        # Stability analysis
        report.append("Stability Analysis")
        report.append("-----------------")
        for variant, stability in metrics['stability'].items():
            report.append(f"{variant}:")
            report.append(f"  Error Stability (std): {stability['error_std']:.4f}")
            if stability['param_std'] is not None:
                report.append(f"  Parameter Stability (std): {stability['param_std']:.4f}")
            report.append("")
        
        # Adaptation speed
        report.append("Adaptation Performance")
        report.append("---------------------")
        for variant, speed in metrics['adaptation_speed'].items():
            if speed is not None:
                report.append(f"{variant}:")
                report.append(f"  Adaptation Lag: {abs(speed)} time steps")
                report.append(f"  Direction: {'Leading' if speed < 0 else 'Lagging'}\n")
        
        # Efficiency comparison
        report.append("Message Efficiency")
        report.append("-----------------")
        efficiencies = metrics['efficiency']
        best_variant = max(efficiencies, key=efficiencies.get)
        report.append(f"Most efficient variant: {best_variant}")
        for variant, efficiency in efficiencies.items():
            report.append(f"  {variant}: {efficiency:.6f}")
        
        if output_file:
            with open(output_file, 'w') as f:
                f.write('\n'.join(report))
        
        return '\n'.join(report)
    
    def run_analysis(self, output_dir=None):
        """Run complete comparative analysis."""
        if output_dir:
            output_dir = Path(output_dir)
            output_dir.mkdir(parents=True, exist_ok=True)
        
        # Load and process results
        results = self.load_results()
        df = self.create_analysis_dataframe(results)
        
        # Calculate metrics and run tests
        metrics = self.calculate_performance_metrics(df)
        tests = self.perform_statistical_tests(df)
        
        # Additional analysis for time-varying conditions
        time_varying_metrics = self.analyze_time_varying_performance(df)
        
        # Generate visualizations
        if output_dir:
            self.plot_convergence_comparison(
                df, output_dir / 'convergence_comparison.png')
            self.plot_efficiency_metrics(
                df, output_dir / 'efficiency_metrics.png')
            self.plot_network_impact(
                df, output_dir / 'network_impact.png')
            self.plot_time_varying_analysis(
                df, output_dir / 'time_varying_analysis.png')
            
            # Generate reports
            report = self.generate_report(
                metrics, tests, output_dir / 'analysis_report.txt')
            time_varying_report = self.generate_time_varying_report(
                time_varying_metrics, output_dir / 'time_varying_report.txt')
        else:
            report = self.generate_report(metrics, tests)
            time_varying_report = self.generate_time_varying_report(time_varying_metrics)
        
        return {
            'metrics': metrics,
            'tests': tests,
            'time_varying_metrics': time_varying_metrics,
            'report': report,
            'time_varying_report': time_varying_report
        }

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Perform comparative analysis of consensus results')
    parser.add_argument('results_dir', help='Directory containing experiment results')
    parser.add_argument('--output-dir', help='Directory to save analysis results')
    
    args = parser.parse_args()
    
    analyzer = ConsensusComparativeAnalysis(args.results_dir)
    analyzer.run_analysis(args.output_dir)

if __name__ == '__main__':
    main()