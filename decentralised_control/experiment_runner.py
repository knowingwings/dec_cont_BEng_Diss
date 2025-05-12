# experiment_runner.py
#!/usr/bin/env python3

import subprocess
import time
import os
import itertools
import argparse
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import stats

class ExperimentRunner:
    """
    Automated runner for executing experiments with different parameter combinations.
    """
    
    def __init__(self, output_dir, trials_per_config=5):
        self.output_dir = output_dir
        self.trials_per_config = trials_per_config
        
        # Control variables and levels
        self.task_counts = [4, 8, 16, 32]
        self.delays = [0, 50, 200, 500]  # ms
        self.packet_loss_probs = [0.0, 0.1, 0.3, 0.5]
        self.epsilons = [0.01, 0.05, 0.2, 0.5]
        
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Log file
        self.log_file = os.path.join(output_dir, 'experiment_log.txt')
        
        # Results dataframe
        self.results = pd.DataFrame()
    
    def log(self, message):
        """Log a message to both console and log file."""
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        log_msg = f'[{timestamp}] {message}'
        
        print(log_msg)
        with open(self.log_file, 'a') as f:
            f.write(log_msg + '\n')
    
    def run_experiment(self, tasks, delay, packet_loss, epsilon, trial):
        """Run a single experiment with the given parameters."""
        experiment_id = f'exp_T{tasks}_D{delay}_P{int(packet_loss*100)}_E{int(epsilon*100)}_trial{trial}'
        
        self.log(f'Starting experiment {experiment_id}')
        
        # Prepare launch command
        cmd = [
            'ros2', 'launch', 'decentralized_control', 'decentralised_controller.launch.py',
            f'num_tasks:={tasks}',
            f'delay_ms:={delay}',
            f'packet_loss:={packet_loss}',
            f'epsilon:={epsilon}',
            f'experiment_id:={experiment_id}'
        ]
        
        # Run experiment
        try:
            self.log(f'Running command: {" ".join(cmd)}')
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            # Wait for experiment completion (with timeout)
            # In a real implementation, would use a more robust approach for determining completion
            timeout = 300  # 5 minutes timeout
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.log(f'Experiment {experiment_id} timed out after {timeout} seconds')
                process.kill()
            
            # Check results file
            results_file = os.path.join(
                self.output_dir, f'metrics_{experiment_id}.json')
            
            if os.path.exists(results_file):
                with open(results_file, 'r') as f:
                    metrics = json.load(f)
                
                self.log(f'Experiment {experiment_id} completed successfully')
                return metrics
            else:
                self.log(f'Error: Results file not found for experiment {experiment_id}')
                return None
                
        except Exception as e:
            self.log(f'Error running experiment {experiment_id}: {str(e)}')
            return None
    
    def run_all_experiments(self):
        """Run all parameter combinations."""
        # Generate all combinations
        parameter_combinations = list(itertools.product(
            self.task_counts, self.delays, self.packet_loss_probs, self.epsilons))
        
        total_experiments = len(parameter_combinations) * self.trials_per_config
        self.log(f'Running {total_experiments} experiments ({len(parameter_combinations)} configurations, {self.trials_per_config} trials each)')
        
        results_list = []
        
        # Run each combination
        for tasks, delay, packet_loss, epsilon in parameter_combinations:
            for trial in range(1, self.trials_per_config + 1):
                metrics = self.run_experiment(tasks, delay, packet_loss, epsilon, trial)
                
                if metrics:
                    # Extract key metrics
                    result = {
                        'num_tasks': tasks,
                        'delay_ms': delay,
                        'packet_loss': packet_loss,
                        'epsilon': epsilon,
                        'trial': trial,
                        'convergence_time': metrics.get('convergence_time'),
                        'makespan': metrics.get('makespan'),
                        'total_messages': metrics.get('total_messages'),
                        'avg_reassignments_per_task': metrics.get('avg_reassignments_per_task')
                    }
                    
                    results_list.append(result)
        
        # Convert to DataFrame
        self.results = pd.DataFrame(results_list)
        
        # Save results
        csv_file = os.path.join(self.output_dir, 'experiment_results.csv')
        self.results.to_csv(csv_file, index=False)
        self.log(f'Results saved to {csv_file}')
        
        return self.results
    
    def analyze_results(self):
        """Analyze experimental results."""
        if self.results.empty:
            self.log('No results available for analysis')
            return
        
        self.log('Analyzing experimental results...')
        
        # Calculate mean and std for each configuration
        grouped = self.results.groupby(['num_tasks', 'delay_ms', 'packet_loss', 'epsilon']).agg(
            convergence_time_mean=('convergence_time', 'mean'),
            convergence_time_std=('convergence_time', 'std'),
            makespan_mean=('makespan', 'mean'),
            makespan_std=('makespan', 'std'),
            total_messages_mean=('total_messages', 'mean'),
            total_messages_std=('total_messages', 'std'),
            avg_reassignments_mean=('avg_reassignments_per_task', 'mean'),
            avg_reassignments_std=('avg_reassignments_per_task', 'std')
        ).reset_index()
        
        # Save summary statistics
        summary_file = os.path.join(self.output_dir, 'results_summary.csv')
        grouped.to_csv(summary_file, index=False)
        
        # Create visualizations
        self.create_visualizations(grouped)
        
        # Perform ANOVA to determine significant factors
        self.perform_anova()
        
        self.log('Analysis completed')
    
    def create_visualizations(self, grouped_results):
        """Create visualizations of results."""
        # Create output directory for plots
        plots_dir = os.path.join(self.output_dir, 'plots')
        os.makedirs(plots_dir, exist_ok=True)
        
        # Plot 1: Effect of number of tasks on convergence time
        plt.figure(figsize=(10, 6))
        for delay in self.delays:
            subset = grouped_results[
                (grouped_results['delay_ms'] == delay) & 
                (grouped_results['packet_loss'] == 0.0) & 
                (grouped_results['epsilon'] == 0.05)
            ]
            plt.errorbar(
                subset['num_tasks'], subset['convergence_time_mean'], 
                yerr=subset['convergence_time_std'],
                label=f'Delay = {delay}ms'
            )
        
        plt.xlabel('Number of Tasks')
        plt.ylabel('Convergence Time (s)')
        plt.title('Effect of Task Count on Convergence Time')
        plt.legend()
        plt.grid(True)
        plt.savefig(os.path.join(plots_dir, 'tasks_vs_convergence.png'))
        
        # Plot 2: Effect of packet loss on message count
        plt.figure(figsize=(10, 6))
        for tasks in self.task_counts:
            subset = grouped_results[
                (grouped_results['num_tasks'] == tasks) & 
                (grouped_results['delay_ms'] == 0) & 
                (grouped_results['epsilon'] == 0.05)
            ]
            plt.errorbar(
                subset['packet_loss'], subset['total_messages_mean'], 
                yerr=subset['total_messages_std'],
                label=f'Tasks = {tasks}'
            )
        
        plt.xlabel('Packet Loss Probability')
        plt.ylabel('Total Messages')
        plt.title('Effect of Packet Loss on Communication Overhead')
        plt.legend()
        plt.grid(True)
        plt.savefig(os.path.join(plots_dir, 'packet_loss_vs_messages.png'))
        
        # Plot 3: Effect of epsilon on reassignments
        plt.figure(figsize=(10, 6))
        for tasks in [8, 16, 32]:
            subset = grouped_results[
                (grouped_results['num_tasks'] == tasks) & 
                (grouped_results['delay_ms'] == 0) & 
                (grouped_results['packet_loss'] == 0.0)
            ]
            plt.errorbar(
                subset['epsilon'], subset['avg_reassignments_mean'], 
                yerr=subset['avg_reassignments_std'],
                label=f'Tasks = {tasks}'
            )
        
        plt.xlabel('Epsilon Value')
        plt.ylabel('Average Reassignments per Task')
        plt.title('Effect of Epsilon on Task Reassignments')
        plt.legend()
        plt.grid(True)
        plt.savefig(os.path.join(plots_dir, 'epsilon_vs_reassignments.png'))
        
        # Plot 4: Interaction between delay and packet loss
        plt.figure(figsize=(12, 8))
        
        for delay in self.delays:
            x = []
            y = []
            for loss in self.packet_loss_probs:
                subset = grouped_results[
                    (grouped_results['delay_ms'] == delay) & 
                    (grouped_results['packet_loss'] == loss) & 
                    (grouped_results['num_tasks'] == 16) &
                    (grouped_results['epsilon'] == 0.05)
                ]
                if not subset.empty:
                    x.append(loss)
                    y.append(subset['makespan_mean'].values[0])
            
            plt.plot(x, y, marker='o', label=f'Delay = {delay}ms')
        
        plt.xlabel('Packet Loss Probability')
        plt.ylabel('Makespan (s)')
        plt.title('Interaction Between Delay and Packet Loss (16 tasks)')
        plt.legend()
        plt.grid(True)
        plt.savefig(os.path.join(plots_dir, 'delay_packet_loss_interaction.png'))
    
    def perform_anova(self):
        """Perform ANOVA analysis to determine significant factors."""
        # Create a linear model with all factors
        from statsmodels.formula.api import ols
        from statsmodels.stats.anova import anova_lm
        
        # Convert factors to categorical where appropriate
        model_data = self.results.copy()
        
        # Fit models for different response variables
        response_variables = ['convergence_time', 'makespan', 'total_messages', 'avg_reassignments_per_task']
        
        anova_results = {}
        
        for response in response_variables:
            formula = f'{response} ~ C(num_tasks) + C(delay_ms) + C(packet_loss) + C(epsilon)'
            model = ols(formula, data=model_data).fit()
            anova_table = anova_lm(model)
            
            anova_results[response] = anova_table
            
            # Save ANOVA results
            anova_file = os.path.join(self.output_dir, f'anova_{response}.csv')
            anova_table.to_csv(anova_file)
            
            self.log(f'ANOVA results for {response} saved to {anova_file}')
            
            # Print significant factors
            significant_factors = anova_table[anova_table['PR(>F)'] < 0.05].index.tolist()
            self.log(f'Significant factors for {response}: {significant_factors}')
        
        return anova_results

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run decentralized control experiments')
    parser.add_argument('--output_dir', type=str, default='/tmp/decentralized_control/results',
                        help='Directory for experiment results')
    parser.add_argument('--trials', type=int, default=5,
                        help='Number of trials per parameter configuration')
    
    args = parser.parse_args()
    
    runner = ExperimentRunner(args.output_dir, args.trials)
    runner.run_all_experiments()
    runner.analyze_results()