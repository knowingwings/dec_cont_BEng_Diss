#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
from pathlib import Path
import json
from scipy import signal, stats
import yaml
import argparse

class AdaptationAnalyzer:
    """Analyzes the adaptation dynamics of network-aware consensus."""
    
    def __init__(self, results_dir):
        self.results_dir = Path(results_dir)
        
        # Set plotting style
        sns.set_style("whitegrid")
        plt.rcParams.update({
            'font.size': 12,
            'figure.figsize': (12, 8),
            'lines.linewidth': 2
        })
    
    def load_network_metrics(self, experiment_id):
        """Load network metrics from file."""
        metric_file = self.results_dir / f'network_metrics_{experiment_id}.json'
        with open(metric_file, 'r') as f:
            return json.load(f)
    
    def analyze_adaptation_response(self, metrics):
        """Analyze system response to network changes."""
        analysis = {
            'time_constants': [],      # System response time constants
            'settling_times': [],      # Time to reach steady state
            'overshoot': [],          # Maximum overshoot percentages
            'steady_state_error': []   # Average steady state error
        }
        
        # Extract time series data
        times = np.array(metrics['timestamps'])
        delays = []
        errors = []
        gammas = []
        
        for robot_id in metrics['network_metrics']:
            delays.extend(metrics['network_metrics'][robot_id]['latencies'])
            gammas.extend(metrics['gamma_values'][robot_id])
        
        delays = np.array(delays)
        errors = np.array(metrics['consensus_errors'])
        gammas = np.array(gammas)
        
        # Find step changes in network delay
        delay_changes = np.where(np.abs(np.diff(delays)) > np.std(delays))[0]
        
        # Analyze response around each change
        for change_idx in delay_changes:
            # Extract response window
            window_size = 100  # samples
            start_idx = max(0, change_idx - 10)
            end_idx = min(len(times), change_idx + window_size)
            
            window_times = times[start_idx:end_idx] - times[start_idx]
            window_errors = errors[start_idx:end_idx]
            window_gammas = gammas[start_idx:end_idx]
            
            # Calculate time constant (time to reach 63.2% of final value)
            final_value = np.mean(window_errors[-20:])  # Average of last 20 samples
            initial_value = window_errors[0]
            target = initial_value + 0.632 * (final_value - initial_value)
            
            try:
                time_constant_idx = np.where(window_errors >= target)[0][0]
                time_constant = window_times[time_constant_idx]
                analysis['time_constants'].append(time_constant)
            except IndexError:
                continue
            
            # Calculate settling time (time to stay within 2% of final value)
            tolerance = 0.02 * (final_value - initial_value)
            settled = np.where(np.abs(window_errors - final_value) <= tolerance)[0]
            
            if len(settled) > 0:
                settling_time = window_times[settled[0]]
                analysis['settling_times'].append(settling_time)
            
            # Calculate maximum overshoot
            if final_value != 0:
                overshoot = 100 * (np.max(window_errors) - final_value) / final_value
                analysis['overshoot'].append(max(0, overshoot))
            
            # Calculate steady state error
            steady_state_error = np.mean(np.abs(window_errors[-20:] - final_value))
            analysis['steady_state_error'].append(steady_state_error)
        
        return analysis
    
    def analyze_adaptation_metrics(self, metrics):
        """Calculate adaptation performance metrics."""
        adaptation_metrics = {}
        
        # Calculate delay tracking correlation
        delays = []
        gammas = []
        for robot_id in metrics['network_metrics']:
            delays.extend(metrics['network_metrics'][robot_id]['latencies'])
            gammas.extend(metrics['gamma_values'][robot_id])
        
        delays = np.array(delays)
        gammas = np.array(gammas)
        errors = np.array(metrics['consensus_errors'])
        
        # Calculate correlation between network delay and adaptation parameter
        delay_gamma_corr = stats.pearsonr(delays, gammas)[0]
        adaptation_metrics['delay_gamma_correlation'] = delay_gamma_corr
        
        # Calculate adaptation speed (using cross-correlation)
        xcorr = signal.correlate(delays, gammas)
        lags = signal.correlation_lags(len(delays), len(gammas))
        adaptation_lag = lags[np.argmax(np.abs(xcorr))]
        adaptation_metrics['adaptation_lag'] = adaptation_lag
        
        # Calculate stability metrics
        adaptation_metrics['gamma_stability'] = np.std(gammas)
        adaptation_metrics['error_stability'] = np.std(errors)
        
        # Calculate convergence rate during stable periods
        stable_periods = self.identify_stable_periods(delays)
        convergence_rates = []
        
        for start, end in stable_periods:
            period_errors = errors[start:end]
            if len(period_errors) > 1:
                rate = -np.polyfit(np.arange(len(period_errors)), 
                                 np.log(period_errors), 1)[0]
                convergence_rates.append(rate)
        
        adaptation_metrics['mean_convergence_rate'] = np.mean(convergence_rates) if convergence_rates else 0
        
        return adaptation_metrics
    
    def identify_stable_periods(self, signal, threshold=0.1):
        """Identify periods of stable network conditions."""
        stable_periods = []
        
        # Calculate signal variation
        variation = np.abs(np.diff(signal))
        stable = variation < (threshold * np.std(signal))
        
        # Find continuous stable regions
        region_start = None
        
        for i, is_stable in enumerate(stable):
            if is_stable and region_start is None:
                region_start = i
            elif not is_stable and region_start is not None:
                if i - region_start > 10:  # Minimum stable period length
                    stable_periods.append((region_start, i))
                region_start = None
        
        return stable_periods
    
    def plot_adaptation_dynamics(self, metrics, output_file=None):
        """Generate comprehensive adaptation dynamics plots."""
        times = np.array(metrics['timestamps'])
        delays = []
        errors = []
        gammas = []
        message_rates = []
        
        for robot_id in metrics['network_metrics']:
            delays.extend(metrics['network_metrics'][robot_id]['latencies'])
            gammas.extend(metrics['gamma_values'][robot_id])
            message_rates.extend(metrics['network_metrics'][robot_id]['message_rates'])
        
        delays = np.array(delays)
        errors = np.array(metrics['consensus_errors'])
        gammas = np.array(gammas)
        message_rates = np.array(message_rates)
        
        plt.figure(figsize=(15, 10))
        
        # Plot 1: Network conditions and adaptation
        plt.subplot(2, 2, 1)
        plt.plot(times, delays, label='Network Delay', alpha=0.7)
        plt.plot(times, gammas, label='Adaptation Parameter', alpha=0.7)
        plt.title('Network Conditions and Adaptation')
        plt.xlabel('Time (s)')
        plt.ylabel('Value')
        plt.legend()
        
        # Plot 2: Consensus error evolution
        plt.subplot(2, 2, 2)
        plt.plot(times, errors)
        plt.title('Consensus Error Evolution')
        plt.xlabel('Time (s)')
        plt.ylabel('Error')
        plt.yscale('log')
        
        # Plot 3: Message rate adaptation
        plt.subplot(2, 2, 3)
        plt.plot(times, message_rates)
        plt.title('Message Rate Adaptation')
        plt.xlabel('Time (s)')
        plt.ylabel('Messages/s')
        
        # Plot 4: Phase portrait (error vs. adaptation parameter)
        plt.subplot(2, 2, 4)
        plt.scatter(gammas, errors, alpha=0.5, s=10)
        plt.title('Phase Portrait')
        plt.xlabel('Adaptation Parameter')
        plt.ylabel('Error')
        plt.yscale('log')
        
        plt.tight_layout()
        
        if output_file:
            plt.savefig(output_file, bbox_inches='tight', dpi=300)
        plt.close()
    
    def generate_adaptation_report(self, metrics, analysis, output_file=None):
        """Generate detailed adaptation analysis report."""
        report = []
        report.append("Network-Aware Consensus Adaptation Analysis")
        report.append("========================================\n")
        
        # System response characteristics
        report.append("System Response Characteristics")
        report.append("-----------------------------")
        report.append(f"Average Time Constant: {np.mean(analysis['time_constants']):.3f}s")
        report.append(f"Average Settling Time: {np.mean(analysis['settling_times']):.3f}s")
        report.append(f"Average Overshoot: {np.mean(analysis['overshoot']):.2f}%")
        report.append(f"Average Steady State Error: {np.mean(analysis['steady_state_error']):.4f}\n")
        
        # Adaptation performance metrics
        adapt_metrics = self.analyze_adaptation_metrics(metrics)
        report.append("Adaptation Performance Metrics")
        report.append("-----------------------------")
        report.append(f"Delay-Gamma Correlation: {adapt_metrics['delay_gamma_correlation']:.4f}")
        report.append(f"Adaptation Lag: {adapt_metrics['adaptation_lag']} samples")
        report.append(f"Parameter Stability (std): {adapt_metrics['gamma_stability']:.4f}")
        report.append(f"Error Stability (std): {adapt_metrics['error_stability']:.4f}")
        report.append(f"Mean Convergence Rate: {adapt_metrics['mean_convergence_rate']:.4f}\n")
        
        # Message efficiency
        total_messages = sum(len(metrics['network_metrics'][rid]['message_rates']) 
                           for rid in metrics['network_metrics'])
        conv_error = np.mean(metrics['consensus_errors'][-20:])  # Final error
        
        report.append("Communication Efficiency")
        report.append("----------------------")
        report.append(f"Total Messages: {total_messages}")
        report.append(f"Final Consensus Error: {conv_error:.6f}")
        report.append(f"Message Efficiency: {1.0/(conv_error * total_messages):.6f}\n")
        
        if output_file:
            with open(output_file, 'w') as f:
                f.write('\n'.join(report))
        
        return '\n'.join(report)
    
    def run_analysis(self, experiment_id, output_dir=None):
        """Run complete adaptation analysis for an experiment."""
        if output_dir:
            output_dir = Path(output_dir)
            output_dir.mkdir(parents=True, exist_ok=True)
        
        # Load metrics
        metrics = self.load_network_metrics(experiment_id)
        
        # Analyze adaptation response
        analysis = self.analyze_adaptation_response(metrics)
        
        # Generate visualizations and report
        if output_dir:
            self.plot_adaptation_dynamics(
                metrics, output_dir / f'adaptation_dynamics_{experiment_id}.png')
            
            report = self.generate_adaptation_report(
                metrics, analysis, 
                output_dir / f'adaptation_analysis_{experiment_id}.txt')
        else:
            report = self.generate_adaptation_report(metrics, analysis)
        
        return {
            'analysis': analysis,
            'adaptation_metrics': self.analyze_adaptation_metrics(metrics),
            'report': report
        }

def main():
    parser = argparse.ArgumentParser(
        description='Analyze network-aware consensus adaptation dynamics')
    parser.add_argument('results_dir', 
                       help='Directory containing experiment results')
    parser.add_argument('experiment_id',
                       help='ID of the experiment to analyze')
    parser.add_argument('--output-dir',
                       help='Directory to save analysis results')
    
    args = parser.parse_args()
    
    analyzer = AdaptationAnalyzer(args.results_dir)
    analyzer.run_analysis(args.experiment_id, args.output_dir)

if __name__ == '__main__':
    main()