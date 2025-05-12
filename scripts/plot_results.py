#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import json
import os
from scipy import stats

class ConsensusVisualizer:
    """Visualizes consensus performance metrics under different network conditions."""
    
    def __init__(self):
        sns.set_style("whitegrid")
        plt.rcParams.update({'font.size': 12})
    
    def load_results(self, results_file):
        """Load experiment results from JSON file."""
        with open(results_file, 'r') as f:
            return json.load(f)
    
    def plot_convergence_comparison(self, results, output_file=None):
        """Plot convergence time comparison across network conditions."""
        conditions = list(results.keys())
        conv_times = [results[cond]['average']['convergence_time'] for cond in conditions]
        
        plt.figure(figsize=(10, 6))
        sns.barplot(x=conditions, y=conv_times)
        plt.title('Consensus Convergence Time by Network Condition')
        plt.xlabel('Network Condition')
        plt.ylabel('Convergence Time (s)')
        plt.xticks(rotation=45)
        
        if output_file:
            plt.savefig(output_file, bbox_inches='tight')
        plt.close()
    
    def plot_message_efficiency(self, results, output_file=None):
        """Plot message efficiency metrics."""
        conditions = list(results.keys())
        messages = [results[cond]['average']['total_messages'] for cond in conditions]
        conv_times = [results[cond]['average']['convergence_time'] for cond in conditions]
        
        efficiency = np.array(conv_times) / np.array(messages)
        
        plt.figure(figsize=(12, 5))
        
        plt.subplot(1, 2, 1)
        sns.barplot(x=conditions, y=messages)
        plt.title('Total Messages')
        plt.xlabel('Network Condition')
        plt.ylabel('Number of Messages')
        plt.xticks(rotation=45)
        
        plt.subplot(1, 2, 2)
        sns.barplot(x=conditions, y=efficiency)
        plt.title('Convergence Efficiency')
        plt.xlabel('Network Condition')
        plt.ylabel('Time per Message (s)')
        plt.xticks(rotation=45)
        
        plt.tight_layout()
        if output_file:
            plt.savefig(output_file, bbox_inches='tight')
        plt.close()
    
    def plot_network_impact(self, results, output_file=None):
        """Plot impact of network conditions on performance."""
        conditions = []
        delays = []
        loss_probs = []
        conv_times = []
        
        for cond, data in results.items():
            for run in data['individual_runs']:
                conditions.append(cond)
                delays.append(run['network_conditions']['delay_mean_ms'])
                loss_probs.append(run['network_conditions']['packet_loss_prob'])
                conv_times.append(run['convergence_time'])
        
        plt.figure(figsize=(12, 5))
        
        plt.subplot(1, 2, 1)
        sns.scatterplot(x=delays, y=conv_times)
        plt.title('Impact of Network Delay')
        plt.xlabel('Mean Delay (ms)')
        plt.ylabel('Convergence Time (s)')
        
        # Add trend line
        z = np.polyfit(delays, conv_times, 1)
        p = np.poly1d(z)
        plt.plot(delays, p(delays), "r--", alpha=0.8)
        
        plt.subplot(1, 2, 2)
        sns.scatterplot(x=loss_probs, y=conv_times)
        plt.title('Impact of Packet Loss')
        plt.xlabel('Packet Loss Probability')
        plt.ylabel('Convergence Time (s)')
        
        # Add trend line
        z = np.polyfit(loss_probs, conv_times, 1)
        p = np.poly1d(z)
        plt.plot(loss_probs, p(loss_probs), "r--", alpha=0.8)
        
        plt.tight_layout()
        if output_file:
            plt.savefig(output_file, bbox_inches='tight')
        plt.close()
    
    def generate_statistical_report(self, results, output_file=None):
        """Generate statistical analysis report."""
        report = []
        conditions = list(results.keys())
        
        # Prepare data for analysis
        condition_data = {}
        for cond in conditions:
            condition_data[cond] = {
                'conv_times': [run['convergence_time'] for run in results[cond]['individual_runs']],
                'messages': [run['total_messages'] for run in results[cond]['individual_runs']]
            }
        
        # Perform statistical tests
        report.append("Statistical Analysis Report")
        report.append("=========================\n")
        
        # ANOVA test for convergence times
        conv_times_groups = [condition_data[cond]['conv_times'] for cond in conditions]
        f_stat, p_val = stats.f_oneway(*conv_times_groups)
        report.append("Convergence Time ANOVA Test")
        report.append(f"F-statistic: {f_stat:.4f}")
        report.append(f"p-value: {p_val:.4f}\n")
        
        # Correlation analysis
        delays = []
        loss_probs = []
        conv_times = []
        
        for cond, data in results.items():
            for run in data['individual_runs']:
                delays.append(run['network_conditions']['delay_mean_ms'])
                loss_probs.append(run['network_conditions']['packet_loss_prob'])
                conv_times.append(run['convergence_time'])
        
        delay_corr, delay_p = stats.pearsonr(delays, conv_times)
        loss_corr, loss_p = stats.pearsonr(loss_probs, conv_times)
        
        report.append("Correlation Analysis")
        report.append(f"Delay vs Convergence Time: r={delay_corr:.4f}, p={delay_p:.4f}")
        report.append(f"Packet Loss vs Convergence Time: r={loss_corr:.4f}, p={loss_p:.4f}\n")
        
        # Summary statistics
        report.append("Summary Statistics")
        report.append("-----------------")
        for cond in conditions:
            conv_times = condition_data[cond]['conv_times']
            messages = condition_data[cond]['messages']
            
            report.append(f"\n{cond}:")
            report.append(f"Convergence Time (s) - Mean: {np.mean(conv_times):.2f}, Std: {np.std(conv_times):.2f}")
            report.append(f"Total Messages - Mean: {np.mean(messages):.2f}, Std: {np.std(messages):.2f}")
        
        if output_file:
            with open(output_file, 'w') as f:
                f.write('\n'.join(report))
        
        return '\n'.join(report)
    
    def analyze_results(self, results_file, output_dir):
        """Perform complete analysis of experiment results."""
        results = self.load_results(results_file)
        
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate plots
        self.plot_convergence_comparison(
            results, os.path.join(output_dir, 'convergence_comparison.png'))
        
        self.plot_message_efficiency(
            results, os.path.join(output_dir, 'message_efficiency.png'))
        
        self.plot_network_impact(
            results, os.path.join(output_dir, 'network_impact.png'))
        
        # Generate statistical report
        self.generate_statistical_report(
            results, os.path.join(output_dir, 'statistical_analysis.txt'))

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Analyze consensus experiment results')
    parser.add_argument('results_file', help='Path to results JSON file')
    parser.add_argument('--output-dir', default='analysis_results',
                      help='Directory to save analysis results')
    
    args = parser.parse_args()
    
    visualizer = ConsensusVisualizer()
    visualizer.analyze_results(args.results_file, args.output_dir)

if __name__ == '__main__':
    main()