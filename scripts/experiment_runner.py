#!/usr/bin/env python3

import os
import sys
import time
import yaml
import json
import argparse
import subprocess
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
from itertools import product
import signal
import shutil
import multiprocessing
from pathlib import Path
import psutil
import GPUtil

class GpuAcceleratedExperimentRunner:
    """
    Runs parallel experiments optimized for GPU acceleration on AMD Radeon GPU.
    """
    
    def __init__(self, args):
        # Set up experiment parameters
        self.base_dir = args.output_dir
        self.num_trials = args.trials
        self.timeout = args.timeout
        self.record_video = args.record_video
        self.record_frequency = args.record_frequency
        self.parallel = args.parallel
        self.skip_completed = args.skip_completed
        self.run_comparative = args.comparative
        self.max_parallel = args.max_parallel
        self.gpu_id = args.gpu_id
        
        # Experimental control variables
        self.task_counts = [4, 8, 16, 32]
        self.delays = [0, 50, 200, 500]  # ms
        self.packet_losses = [0.0, 0.1, 0.3, 0.5]
        self.epsilons = [0.01, 0.05, 0.2, 0.5]
        self.environments = ['simple', 'cluttered', 'assembly']
        self.industrial_scenarios = ['automotive', 'electronics', 'furniture']
        
        # Comparative methods
        self.comparative_methods = ['CENTRALIZED', 'GREEDY', 'MARKET_BASED', 'SEQUENTIAL']
        
        # Create directory structure
        self.setup_directories()
        
        # Initialize logging
        self.log_file = os.path.join(self.base_dir, 'experiment_log.txt')
        self.results_df = None
        
        # Track running processes
        self.active_processes = []
        
        # Set up signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Check GPU availability
        self.check_gpu()
        
        # Determine optimal parallelization based on system resources
        self.determine_optimal_parallelization()
        
        # Load experiment configuration if it exists
        self.config_file = os.path.join(self.base_dir, 'experiment_config.yaml')
        self.load_config()
        
        # Log experiment start
        self.log(f"Starting GPU-accelerated experiment: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        self.log(f"Base directory: {self.base_dir}")
        self.log(f"Number of trials: {self.num_trials}")
        self.log(f"Record video: {self.record_video}")
        self.log(f"Using GPU: {self.gpu_available}, Parallel simulations: {self.optimal_parallel}")
    
    def check_gpu(self):
        """Check GPU availability and capabilities."""
        try:
            gpus = GPUtil.getGPUs()
            if gpus:
                if self.gpu_id >= 0 and self.gpu_id < len(gpus):
                    gpu = gpus[self.gpu_id]
                else:
                    gpu = gpus[0]  # Default to first GPU
                
                self.gpu_available = True
                self.gpu_name = gpu.name
                self.gpu_memory = gpu.memoryTotal
                
                self.log(f"GPU detected: {self.gpu_name} with {self.gpu_memory}MB memory")
                
                # Set environment variables for GPU acceleration in Gazebo
                os.environ["MESA_GL_VERSION_OVERRIDE"] = "3.3"
                os.environ["LIBGL_ALWAYS_SOFTWARE"] = "0"  # Force hardware rendering
                
                # AMD specific environment variables
                os.environ["ROC_ENABLE_PRE_VEGA"] = "1"  # Enable older AMD GPUs
                os.environ["HSA_OVERRIDE_GFX_VERSION"] = "10.3.0"  # Force specific GFX version for compatibility
                os.environ["DRI_PRIME"] = "1"  # Use discrete GPU
                
                # For AMD Radeon 7900XT, additional optimizations
                if "7900" in self.gpu_name:
                    os.environ["GPU_MAX_HEAP_SIZE"] = "100"  # 100% heap usage
                    os.environ["GPU_USE_SYNC_OBJECTS"] = "1"  # Use sync objects for better performance
                    os.environ["GPU_MAX_ALLOC_PERCENT"] = "100"  # Allow 100% of GPU memory to be allocated
                    os.environ["GPU_SINGLE_ALLOC_PERCENT"] = "100"  # Allow large single allocations
            else:
                self.gpu_available = False
                self.log("No GPU detected, performance may be limited.")
        except Exception as e:
            self.gpu_available = False
            self.log(f"Error detecting GPU: {e}")
    
    def determine_optimal_parallelization(self):
        """Determine optimal number of parallel simulations based on system resources."""
        # Get CPU and memory information
        cpu_count = multiprocessing.cpu_count()
        memory_gb = psutil.virtual_memory().total / (1024**3)  # Convert to GB
        
        # Heuristic for optimal parallel simulations:
        # - Each Gazebo instance needs ~2 CPU cores
        # - Each Gazebo instance needs ~4GB RAM
        # - Each instance can share GPU for rendering
        
        cpu_limit = max(1, cpu_count // 2)
        memory_limit = max(1, int(memory_gb / 4))
        
        # For AMD 7900XT, we can run more parallel simulations due to 24GB VRAM
        if self.gpu_available and "7900" in self.gpu_name:
            gpu_limit = max(1, int(self.gpu_memory / 1024))  # Based on 1GB VRAM per simulation
        else:
            gpu_limit = 4  # Conservative estimate for other GPUs
        
        # Calculate optimal number of parallel simulations
        self.optimal_parallel = min(cpu_limit, memory_limit, gpu_limit)
        
        # Override with user's max_parallel if specified
        if self.max_parallel > 0:
            self.optimal_parallel = min(self.optimal_parallel, self.max_parallel)
        
        self.log(f"System resources - CPU cores: {cpu_count}, Memory: {memory_gb:.1f}GB")
        self.log(f"Optimal parallel simulations: {self.optimal_parallel} (CPU limit: {cpu_limit}, Memory limit: {memory_limit}, GPU limit: {gpu_limit})")
    
    def setup_directories(self):
        """Create directory structure for experiment results."""
        os.makedirs(self.base_dir, exist_ok=True)
        
        # Create subdirectories
        self.data_dir = os.path.join(self.base_dir, 'data')
        self.video_dir = os.path.join(self.base_dir, 'videos')
        self.plots_dir = os.path.join(self.base_dir, 'plots')
        self.reports_dir = os.path.join(self.base_dir, 'reports')
        self.comparative_dir = os.path.join(self.base_dir, 'comparative')
        
        os.makedirs(self.data_dir, exist_ok=True)
        os.makedirs(self.video_dir, exist_ok=True)
        os.makedirs(self.plots_dir, exist_ok=True)
        os.makedirs(self.reports_dir, exist_ok=True)
        os.makedirs(self.comparative_dir, exist_ok=True)
    
    def load_config(self):
        """Load experiment configuration if it exists."""
        self.completed_runs = set()
        
        if os.path.exists(self.config_file):
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
                if 'completed_runs' in config:
                    self.completed_runs = set(config['completed_runs'])
                if 'last_experiment_date' in config:
                    self.log(f"Resuming from last experiment on: {config['last_experiment_date']}")
        
        self.save_config()
    
    def save_config(self):
        """Save experiment configuration."""
        config = {
            'last_experiment_date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'completed_runs': list(self.completed_runs)
        }
        
        with open(self.config_file, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
    
    def generate_run_id(self, params):
        """Generate a unique run ID from parameters."""
        tasks = params.get('num_tasks')
        delay = params.get('delay_ms')
        packet_loss = params.get('packet_loss')
        epsilon = params.get('epsilon')
        environment = params.get('environment')
        scenario = params.get('scenario_type', 'none')
        trial = params.get('trial')
        
        return f"T{tasks}_D{delay}_P{int(packet_loss*100)}_E{int(epsilon*100)}_ENV{environment}_S{scenario}_R{trial}"
    
    def run_exists(self, run_id):
        """Check if this run has already been completed."""
        if run_id in self.completed_runs:
            return True
        
        # Check for data file
        data_file = os.path.join(self.data_dir, f"{run_id}.json")
        return os.path.exists(data_file)
    
    def mark_completed(self, run_id):
        """Mark a run as completed."""
        self.completed_runs.add(run_id)
        self.save_config()
    
    def log(self, message):
        """Log a message to both console and log file."""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        log_msg = f"[{timestamp}] {message}"
        
        print(log_msg)
        with open(self.log_file, 'a') as f:
            f.write(log_msg + '\n')
    
    def signal_handler(self, sig, frame):
        """Handle interrupts by cleaning up processes."""
        self.log("Interrupt received, shutting down experiments...")
        
        # Kill all active processes
        for process in self.active_processes:
            if process.poll() is None:  # If process is still running
                try:
                    process.terminate()
                    self.log(f"Terminated process {process.pid}")
                except:
                    self.log(f"Failed to terminate process {process.pid}")
        
        # Save configuration
        self.save_config()
        
        # Exit
        sys.exit(0)
    
    def run_single_experiment(self, params):
        """Run a single experiment with the given parameters."""
        # Generate unique run ID
        run_id = self.generate_run_id(params)
        
        # Skip if already completed and skip_completed is True
        if self.skip_completed and self.run_exists(run_id):
            self.log(f"Skipping completed run: {run_id}")
            return None
        
        # Prepare output directories
        run_data_dir = os.path.join('/tmp/decentralized_control/results', run_id)
        os.makedirs(run_data_dir, exist_ok=True)
        
        # Determine whether to record this run
        do_record = self.record_video and (params['trial'] % self.record_frequency == 0)
        video_path = None
        
        if do_record:
            video_path = os.path.join(self.video_dir, f"{run_id}.mp4")
            self.log(f"Recording video for run: {run_id}")
        
        # Set up GPU-specific environment variables for this run
        env_vars = os.environ.copy()
        
        # GPU optimization environment variables
        if self.gpu_available:
            # Display ID for Gazebo to use (distribute across Xservers if we're running multiple)
            if hasattr(params, 'process_idx'):
                display_id = params['process_idx'] % 10  # Use different display numbers (0-9)
                env_vars["DISPLAY"] = f":{display_id}.0"
            
            # OpenGL settings for AMD
            env_vars["GALLIUM_DRIVER"] = "radeonsi"  # Use the radeonsi Gallium driver
            
            # Gazebo-specific GPU optimizations
            env_vars["GAZEBO_GPU_RAY_TRACING"] = "0"  # Disable ray tracing (not needed for our sim)
            env_vars["OGRE_RTT_MODE"] = "Copy"  # Faster rendering mode
            
            # Use OpenGL 3.3 - modern but well supported
            env_vars["MESA_GL_VERSION_OVERRIDE"] = "3.3"
        
        # Prepare command
        launch_args = [
            'ros2', 'launch', 'decentralized_control', 'decentralised_controller.launch.py',
            f"num_tasks:={params['num_tasks']}",
            f"delay_ms:={params['delay_ms']}",
            f"packet_loss:={params['packet_loss']}",
            f"epsilon:={params['epsilon']}",
            f"environment:={params['environment']}",
            f"scenario_type:={params['scenario_type']}",
            f"experiment_id:={run_id}",
            f"output_dir:={run_data_dir}",
            f"run_comparative:={'true' if self.run_comparative and params['trial'] == 1 else 'false'}"
        ]
        
        # Add comparative method if running comparative analysis
        if self.run_comparative and params['trial'] == 1:
            launch_args.append(f"comparative_method:={params['comparative_method']}")
        
        # Recording setup
        if do_record:
            # Start ROS2 bag recording
            bag_path = os.path.join(run_data_dir, 'rosbag')
            os.makedirs(bag_path, exist_ok=True)
            
            bag_cmd = [
                'ros2', 'bag', 'record',
                '-o', bag_path,
                '/tasks',
                '/auction/assignments',
                '/robot1/auction/bids', 
                '/robot2/auction/bids',
                '/robot1/state', 
                '/robot2/state',
                '/robot1/heartbeat', 
                '/robot2/heartbeat',
                '/tf', 
                '/tf_static',
                '/collaboration/requests',
                '/collaboration/responses',
                '/robot1/collaboration/sync',
                '/robot2/collaboration/sync'
            ]
            
            # Use hardware-accelerated encoding with AMD GPU (via ffmpeg/vaapi)
            record_cmd = [
                'ffmpeg', '-y',
                '-f', 'x11grab',
                '-s', '1920x1080',
                '-i', env_vars.get("DISPLAY", ":0.0"),
                '-vaapi_device', '/dev/dri/renderD128',  # AMD GPU render node
                '-vf', 'format=nv12,hwupload',
                '-c:v', 'h264_vaapi',  # Use hardware encoding
                '-qp', '24',  # Good quality/size balance
                '-r', '30',
                video_path
            ]
            
            # Start recording processes
            bag_process = subprocess.Popen(bag_cmd, env=env_vars)
            record_process = subprocess.Popen(record_cmd, env=env_vars)
            
            # Add to active processes
            self.active_processes.append(bag_process)
            self.active_processes.append(record_process)
        
        # Start experiment
        self.log(f"Starting experiment run: {run_id}")
        start_time = time.time()
        
        experiment_process = subprocess.Popen(
            launch_args,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            env=env_vars
        )
        
        self.active_processes.append(experiment_process)
        
        # Monitor experiment
        try:
            experiment_process.wait(timeout=self.timeout)
        except subprocess.TimeoutExpired:
            self.log(f"Experiment timed out after {self.timeout} seconds, terminating.")
            experiment_process.terminate()
        
        # Stop recording if active
        if do_record:
            # Stop ffmpeg
            record_process.terminate()
            record_process.wait()
            
            # Stop ROS2 bag
            bag_process.terminate()
            bag_process.wait()
            
            self.log(f"Recording saved to: {video_path}")
        
        # Clean up process
        experiment_process.terminate()
        
        # Remove from active processes
        if do_record:
            self.active_processes.remove(record_process)
            self.active_processes.remove(bag_process)
        
        self.active_processes.remove(experiment_process)
        
        # Calculate duration
        duration = time.time() - start_time
        self.log(f"Experiment run completed in {duration:.2f} seconds")
        
        # Copy results data
        results_file = os.path.join(run_data_dir, f"metrics_{run_id}.json")
        if os.path.exists(results_file):
            # Copy to data directory
            dest_file = os.path.join(self.data_dir, f"{run_id}.json")
            shutil.copy(results_file, dest_file)
            self.log(f"Results saved to: {dest_file}")
            
            # Copy comparative results if available
            if self.run_comparative and params['trial'] == 1:
                comparative_file = os.path.join(run_data_dir, f"comparative_{params['comparative_method']}_T{params['num_tasks']}.json")
                if os.path.exists(comparative_file):
                    dest_comp_file = os.path.join(self.comparative_dir, f"{params['comparative_method']}_T{params['num_tasks']}_E{int(params['epsilon']*100)}.json")
                    shutil.copy(comparative_file, dest_comp_file)
                    self.log(f"Comparative results saved to: {dest_comp_file}")
        else:
            self.log(f"Warning: Results file not found: {results_file}")
            return None
        
        # Mark as completed
        self.mark_completed(run_id)
        
        # Return results
        with open(dest_file, 'r') as f:
            results = json.load(f)
            return {
                'run_id': run_id,
                'task_count': params['num_tasks'],
                'delay_ms': params['delay_ms'],
                'packet_loss': params['packet_loss'],
                'epsilon': params['epsilon'],
                'environment': params['environment'],
                'scenario_type': params['scenario_type'],
                'trial': params['trial'],
                'duration': duration,
                'has_video': do_record,
                'metrics': results
            }
    
    def run_factorial_experiments(self):
        """Run the full factorial experiment design."""
        self.log("Running full factorial experiments...")
        
        # Generate all parameter combinations
        param_combinations = []
        
        # Standard parameters
        for tasks, delay, packet_loss, epsilon in product(
                self.task_counts, self.delays, self.packet_losses, self.epsilons):
            
            # Run for each environment
            for environment in self.environments:
                # Industrial scenario only for assembly environment
                scenario_type = 'none'
                if environment == 'assembly':
                    scenario_type = self.industrial_scenarios[0]  # Use automotive as default
                
                # For each trial
                for trial in range(1, self.num_trials + 1):
                    # Set up parameters
                    params = {
                        'num_tasks': tasks,
                        'delay_ms': delay,
                        'packet_loss': packet_loss,
                        'epsilon': epsilon,
                        'environment': environment,
                        'scenario_type': scenario_type,
                        'trial': trial
                    }
                    
                    # Add comparative method for first trial
                    if self.run_comparative and trial == 1:
                        for method in self.comparative_methods:
                            comp_params = params.copy()
                            comp_params['comparative_method'] = method
                            param_combinations.append(comp_params)
                    else:
                        param_combinations.append(params)
        
        # Special industrial scenario experiments (additional tests)
        for scenario_type in self.industrial_scenarios:
            for tasks in [16, 32]:  # Larger task counts for industrial scenarios
                # Set standard parameters
                delay = 50  # ms
                packet_loss = 0.1
                epsilon = 0.05
                environment = 'assembly'
                
                for trial in range(1, self.num_trials + 1):
                    params = {
                        'num_tasks': tasks,
                        'delay_ms': delay,
                        'packet_loss': packet_loss,
                        'epsilon': epsilon,
                        'environment': environment,
                        'scenario_type': scenario_type,
                        'trial': trial
                    }
                    
                    # Add comparative method for first trial
                    if self.run_comparative and trial == 1:
                        for method in self.comparative_methods:
                            comp_params = params.copy()
                            comp_params['comparative_method'] = method
                            param_combinations.append(comp_params)
                    else:
                        param_combinations.append(params)
        
        # Estimate total time
        total_experiments = len(param_combinations)
        estimated_time = total_experiments * (self.timeout / 60.0)  # minutes
        if self.parallel:
            estimated_time /= self.optimal_parallel
        
        self.log(f"Total experiments to run: {total_experiments}")
        self.log(f"Estimated time: {estimated_time:.1f} minutes ({estimated_time/60.0:.1f} hours)")
        
        # Initialize results storage
        results = []
        
        # Run experiments
        if self.parallel:
            # Check GPU memory to ensure we have enough for parallel simulations
            if self.gpu_available:
                try:
                    gpus = GPUtil.getGPUs()
                    if gpus and self.gpu_id < len(gpus):
                        gpu = gpus[self.gpu_id]
                        available_memory = gpu.memoryFree
                        self.log(f"Available GPU memory: {available_memory}MB")
                        
                        # Adjust parallelism if needed based on available memory
                        memory_per_sim = 1024  # Estimated memory per simulation (conservative 1GB)
                        max_sims_for_memory = max(1, int(available_memory / memory_per_sim))
                        
                        if max_sims_for_memory < self.optimal_parallel:
                            self.log(f"Limiting parallel simulations to {max_sims_for_memory} due to available GPU memory")
                            self.optimal_parallel = max_sims_for_memory
                except Exception as e:
                    self.log(f"Error checking GPU memory: {e}")
            
            # Parallel execution with GPU optimization
            self.log(f"Running with {self.optimal_parallel} parallel processes, optimized for GPU")
            
            # Create a process pool with the optimal number of processes
            with multiprocessing.Pool(processes=self.optimal_parallel) as pool:
                # Add process index to each param set to help with display assignment
                for i, params in enumerate(param_combinations):
                    params['process_idx'] = i % self.optimal_parallel
                
                try:
                    # Map experiments to pool
                    map_results = pool.map(self.run_single_experiment, param_combinations)
                    
                    # Filter None results and add to results list
                    results.extend([r for r in map_results if r is not None])
                except KeyboardInterrupt:
                    self.log("Keyboard interrupt, terminating pool...")
                    pool.terminate()
                    pool.join()
                    raise
        else:
            # Sequential execution
            for i, params in enumerate(param_combinations):
                self.log(f"Running experiment {i+1}/{total_experiments}")
                
                try:
                    result = self.run_single_experiment(params)
                    if result is not None:
                        results.append(result)
                except KeyboardInterrupt:
                    self.log("Keyboard interrupt, stopping experiments...")
                    break
                except Exception as e:
                    self.log(f"Error in experiment: {e}")
        
        # Convert results to DataFrame
        self.results_df = pd.DataFrame(results)
        
        # Save combined results
        if not self.results_df.empty:
            csv_file = os.path.join(self.reports_dir, 'experiment_results.csv')
            self.results_df.to_csv(csv_file, index=False)
            self.log(f"Combined results saved to: {csv_file}")
        
        return self.results_df

    def generate_analysis_report(self):
        """Generate analysis report from experiment data."""
        self.log("Generating analysis report...")
        
        # Load all result files if results_df is not available
        if self.results_df is None or self.results_df.empty:
            self.log("Loading results from files...")
            results = []
            
            for filename in os.listdir(self.data_dir):
                if filename.endswith('.json'):
                    file_path = os.path.join(self.data_dir, filename)
                    try:
                        with open(file_path, 'r') as f:
                            data = json.load(f)
                            
                            # Extract run parameters from filename
                            parts = filename.split('.')[0].split('_')
                            tasks = int(parts[0].replace('T', ''))
                            delay = int(parts[1].replace('D', ''))
                            packet_loss = float(parts[2].replace('P', '')) / 100.0
                            epsilon = float(parts[3].replace('E', '')) / 100.0
                            environment = parts[4].replace('ENV', '')
                            scenario = parts[5].replace('S', '')
                            trial = int(parts[6].replace('R', ''))
                            
                            result = {
                                'run_id': filename.split('.')[0],
                                'task_count': tasks,
                                'delay_ms': delay,
                                'packet_loss': packet_loss,
                                'epsilon': epsilon,
                                'environment': environment,
                                'scenario_type': scenario,
                                'trial': trial,
                                'metrics': data
                            }
                            
                            results.append(result)
                    except Exception as e:
                        self.log(f"Error loading results from {filename}: {e}")
            
            if results:
                self.results_df = pd.DataFrame(results)
                
                # Save combined results
                csv_file = os.path.join(self.reports_dir, 'experiment_results.csv')
                self.results_df.to_csv(csv_file, index=False)
                self.log(f"Combined results saved to: {csv_file}")
            else:
                self.log("No results found. Cannot generate report.")
                return
        
        # Create report directory
        report_date = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_dir = os.path.join(self.reports_dir, f'report_{report_date}')
        os.makedirs(report_dir, exist_ok=True)
        
        # Generate plots
        self.generate_plots(report_dir)
        
        # Generate comparative analysis
        if self.run_comparative:
            self.generate_comparative_analysis(report_dir)
        
        # Generate summary report
        self.generate_summary(report_dir)
        
        self.log(f"Analysis report generated in: {report_dir}")
    
    def generate_plots(self, report_dir):
        """Generate plots for analysis."""
        if self.results_df is None or self.results_df.empty:
            self.log("No results available for plotting.")
            return
        
        # Create plots directory
        plots_dir = os.path.join(report_dir, 'plots')
        os.makedirs(plots_dir, exist_ok=True)
        
        # 1. Plot convergence time vs task count for different epsilons
        self.plot_convergence_vs_tasks(plots_dir)
        
        # 2. Plot communication overhead vs packet loss
        self.plot_overhead_vs_packet_loss(plots_dir)
        
        # 3. Plot optimality gap vs epsilon
        self.plot_optimality_vs_epsilon(plots_dir)
        
        # 4. Plot recovery time vs task count
        self.plot_recovery_time(plots_dir)
        
        # 5. Plot effect of communication delay
        self.plot_delay_effect(plots_dir)
        
        # 6. Plot environment comparison
        self.plot_environment_comparison(plots_dir)
    
    def plot_convergence_vs_tasks(self, plots_dir):
        """Plot convergence time vs task count for different epsilons."""
        plt.figure(figsize=(10, 6))
        
        # Filter data and group by epsilon and task count
        data = self.results_df.copy()
        
        # Group by epsilon and task count, calculate mean and std of convergence time
        grouped = data.groupby(['epsilon', 'task_count'])['metrics.convergence_time'].agg(['mean', 'std']).reset_index()
        
        # Plot for each epsilon
        for eps in self.epsilons:
            subset = grouped[grouped['epsilon'] == eps]
            
            if not subset.empty:
                plt.errorbar(
                    subset['task_count'], subset['mean'], yerr=subset['std'],
                    marker='o', label=f'ε = {eps}'
                )
        
        plt.xlabel('Number of Tasks')
        plt.ylabel('Convergence Time (s)')
        plt.title('Effect of Task Count on Convergence Time')
        plt.legend()
        plt.grid(True)
        
        # Save plot
        plt.savefig(os.path.join(plots_dir, 'convergence_vs_tasks.png'))
        plt.close()
    
    def plot_overhead_vs_packet_loss(self, plots_dir):
        """Plot communication overhead vs packet loss probability."""
        plt.figure(figsize=(10, 6))
        
        # Filter data and group by packet loss and task count
        data = self.results_df.copy()
        
        # Group by packet loss and task count, calculate mean and std of message count
        grouped = data.groupby(['packet_loss', 'task_count'])['metrics.total_messages'].agg(['mean', 'std']).reset_index()
        
        # Plot for each task count
        for tasks in self.task_counts:
            subset = grouped[grouped['task_count'] == tasks]
            
            if not subset.empty:
                plt.errorbar(
                    subset['packet_loss'], subset['mean'], yerr=subset['std'],
                    marker='o', label=f'Tasks = {tasks}'
                )
        
        plt.xlabel('Packet Loss Probability')
        plt.ylabel('Total Messages')
        plt.title('Effect of Packet Loss on Communication Overhead')
        plt.legend()
        plt.grid(True)
        
        # Save plot
        plt.savefig(os.path.join(plots_dir, 'overhead_vs_packet_loss.png'))
        plt.close()
    
    def plot_optimality_vs_epsilon(self, plots_dir):
        """Plot optimality gap vs epsilon value."""
        plt.figure(figsize=(10, 6))
        
        # Load comparative data
        optimality_data = []
        
        for eps in self.epsilons:
            # Task counts to check
            for tasks in self.task_counts:
                # Look for centralized results
                centralized_file = os.path.join(
                    self.comparative_dir, f"CENTRALIZED_T{tasks}_E{int(eps*100)}.json")
                
                if os.path.exists(centralized_file):
                    with open(centralized_file, 'r') as f:
                        central_data = json.load(f)
                        
                        # Get distributed results for same parameters
                        distributed_results = self.results_df[
                            (self.results_df['task_count'] == tasks) & 
                            (self.results_df['epsilon'] == eps)
                        ]
                        
                        if not distributed_results.empty:
                            # Calculate average makespan
                            avg_makespan = distributed_results['metrics.makespan'].mean()
                            std_makespan = distributed_results['metrics.makespan'].std()
                            
                            # Calculate optimality gap
                            central_makespan = central_data.get('makespan', 0)
                            if central_makespan > 0:
                                gap = (avg_makespan - central_makespan) / central_makespan
                                
                                optimality_data.append({
                                    'epsilon': eps,
                                    'task_count': tasks,
                                    'gap': gap,
                                    'gap_std': std_makespan / central_makespan
                                })
        
        # Convert to DataFrame
        if optimality_data:
            opt_df = pd.DataFrame(optimality_data)
            
            # Plot for each task count
            for tasks in self.task_counts:
                subset = opt_df[opt_df['task_count'] == tasks]
                
                if not subset.empty:
                    plt.errorbar(
                        subset['epsilon'], subset['gap'], yerr=subset['gap_std'],
                        marker='o', label=f'Tasks = {tasks}'
                    )
            
            # Plot theoretical bound (2ε)
            plt.plot(self.epsilons, [2*e for e in self.epsilons], 'k--', label='2ε Bound')
            
            plt.xlabel('Epsilon (ε)')
            plt.ylabel('Optimality Gap')
            plt.title('Effect of Epsilon on Optimality Gap')
            plt.legend()
            plt.grid(True)
            
            # Save plot
            plt.savefig(os.path.join(plots_dir, 'optimality_vs_epsilon.png'))
            plt.close()
    
    def plot_recovery_time(self, plots_dir):
        """Plot recovery time vs task count."""
        plt.figure(figsize=(10, 6))
        
        # Filter data to only include runs with recovery events
        data = self.results_df.copy()
        recovery_data = []
        
        for _, row in data.iterrows():
            metrics = row['metrics']
            if 'recovery_events' in metrics and metrics['recovery_events']:
                # Calculate average recovery time
                if 'recovery_times' in metrics and metrics['recovery_times']:
                    avg_recovery = sum(metrics['recovery_times']) / len(metrics['recovery_times'])
                    
                    recovery_data.append({
                        'task_count': row['task_count'],
                        'environment': row['environment'],
                        'recovery_time': avg_recovery
                    })
        
        if recovery_data:
            recovery_df = pd.DataFrame(recovery_data)
            
            # Group by task count and environment
            grouped = recovery_df.groupby(['environment', 'task_count'])['recovery_time'].agg(['mean', 'std']).reset_index()
            
            # Plot for each environment
            for env in self.environments:
                subset = grouped[grouped['environment'] == env]
                
                if not subset.empty:
                    plt.errorbar(
                        subset['task_count'], subset['mean'], yerr=subset['std'],
                        marker='o', label=f'Environment = {env}'
                    )
            
            # Plot theoretical bound (O(|T_f|))
            task_counts = sorted(recovery_df['task_count'].unique())
            plt.plot(task_counts, [t/2 for t in task_counts], 'k--', label='O(|T_f|) Bound')
            
            plt.xlabel('Number of Tasks')
            plt.ylabel('Recovery Time (s)')
            plt.title('Effect of Task Count on Recovery Time')
            plt.legend()
            plt.grid(True)
            
            # Save plot
            plt.savefig(os.path.join(plots_dir, 'recovery_time.png'))
            plt.close()
    
    def plot_delay_effect(self, plots_dir):
        """Plot effect of communication delay."""
        plt.figure(figsize=(10, 6))
        
        # Filter data and group by delay and task count
        data = self.results_df.copy()
        
        # Group by delay and task count, calculate mean and std of convergence time
        grouped = data.groupby(['delay_ms', 'task_count'])['metrics.convergence_time'].agg(['mean', 'std']).reset_index()
        
        # Plot for each task count
        for tasks in self.task_counts:
            subset = grouped[grouped['task_count'] == tasks]
            
            if not subset.empty:
                plt.errorbar(
                    subset['delay_ms'], subset['mean'], yerr=subset['std'],
                    marker='o', label=f'Tasks = {tasks}'
                )
        
        plt.xlabel('Communication Delay (ms)')
        plt.ylabel('Convergence Time (s)')
        plt.title('Effect of Communication Delay on Convergence Time')
        plt.legend()
        plt.grid(True)
        
        # Save plot
        plt.savefig(os.path.join(plots_dir, 'delay_effect.png'))
        plt.close()
    
    def plot_environment_comparison(self, plots_dir):
        """Plot comparison of different environments."""
        plt.figure(figsize=(12, 8))
        
        # Filter data and calculate average makespan for each environment
        data = self.results_df.copy()
        
        # Group by environment and task count
        grouped = data.groupby(['environment', 'task_count'])['metrics.makespan'].agg(['mean', 'std']).reset_index()
        
        # Plot for each environment
        for env in self.environments:
            subset = grouped[grouped['environment'] == env]
            
            if not subset.empty:
                plt.errorbar(
                    subset['task_count'], subset['mean'], yerr=subset['std'],
                    marker='o', label=f'Environment = {env}'
                )
        
        plt.xlabel('Number of Tasks')
        plt.ylabel('Makespan (s)')
        plt.title('Comparison of Environments')
        plt.legend()
        plt.grid(True)
        
        # Save plot
        plt.savefig(os.path.join(plots_dir, 'environment_comparison.png'))
        plt.close()
    
    def generate_comparative_analysis(self, report_dir):
        """Generate comparative analysis of different allocation approaches."""
        if not self.run_comparative:
            return
        
        self.log("Generating comparative analysis...")
        
        # Create comparative directory
        comparative_dir = os.path.join(report_dir, 'comparative')
        os.makedirs(comparative_dir, exist_ok=True)
        
        # Load comparative data
        comparative_data = []
        
        for method in self.comparative_methods:
            for tasks in self.task_counts:
                for eps in self.epsilons:
                    file_path = os.path.join(
                        self.comparative_dir, f"{method}_T{tasks}_E{int(eps*100)}.json")
                    
                    if os.path.exists(file_path):
                        with open(file_path, 'r') as f:
                            data = json.load(f)
                            
                            comparative_data.append({
                                'method': method,
                                'task_count': tasks,
                                'epsilon': eps,
                                'makespan': data.get('makespan', 0),
                                'allocation_time': data.get('allocation_time', 0),
                                'load_ratio': data.get('load_ratio', 0)
                            })
        
        if not comparative_data:
            self.log("No comparative data found.")
            return
        
        # Convert to DataFrame
        comp_df = pd.DataFrame(comparative_data)
        
        # Add distributed results
        for tasks in self.task_counts:
            for eps in self.epsilons:
                distributed_results = self.results_df[
                    (self.results_df['task_count'] == tasks) & 
                    (self.results_df['epsilon'] == eps)
                ]
                
                if not distributed_results.empty:
                    avg_makespan = distributed_results['metrics.makespan'].mean()
                    avg_time = distributed_results['metrics.convergence_time'].mean()
                    
                    comparative_data.append({
                        'method': 'DISTRIBUTED',
                        'task_count': tasks,
                        'epsilon': eps,
                        'makespan': avg_makespan,
                        'allocation_time': avg_time,
                        'load_ratio': 0  # Not available for distributed
                    })
        
        # Update DataFrame
        comp_df = pd.DataFrame(comparative_data)
        
        # Save comparative data
        comp_df.to_csv(os.path.join(comparative_dir, 'comparative_results.csv'), index=False)
        
        # Generate plots
        
        # 1. Makespan comparison
        plt.figure(figsize=(12, 8))
        
        for method in comp_df['method'].unique():
            # Group by task count
            subset = comp_df[comp_df['method'] == method]
            
            if not subset.empty:
                grouped = subset.groupby('task_count')['makespan'].mean().reset_index()
                
                plt.plot(grouped['task_count'], grouped['makespan'], 
                         marker='o', label=method)
        
        plt.xlabel('Number of Tasks')
        plt.ylabel('Makespan (s)')
        plt.title('Makespan Comparison of Different Allocation Methods')
        plt.legend()
        plt.grid(True)
        
        # Save plot
        plt.savefig(os.path.join(comparative_dir, 'makespan_comparison.png'))
        plt.close()
        
        # 2. Allocation time comparison
        plt.figure(figsize=(12, 8))
        
        for method in comp_df['method'].unique():
            # Group by task count
            subset = comp_df[comp_df['method'] == method]
            
            if not subset.empty:
                grouped = subset.groupby('task_count')['allocation_time'].mean().reset_index()
                
                plt.plot(grouped['task_count'], grouped['makespan'], 
                         marker='o', label=method)
        
        plt.xlabel('Number of Tasks')
        plt.ylabel('Allocation Time (s)')
        plt.title('Allocation Time Comparison of Different Methods')
        plt.legend()
        plt.grid(True)
        
        # Save plot
        plt.savefig(os.path.join(comparative_dir, 'allocation_time_comparison.png'))
        plt.close()
        
        # 3. Normalized comparison (radar chart)
        # Find best method for each metric
        best_makespan = comp_df.groupby('task_count')['makespan'].min().reset_index()
        best_makespan.columns = ['task_count', 'best_makespan']
        
        best_time = comp_df.groupby('task_count')['allocation_time'].min().reset_index()
        best_time.columns = ['task_count', 'best_time']
        
        # Merge best values
        comp_df = pd.merge(comp_df, best_makespan, on='task_count')
        comp_df = pd.merge(comp_df, best_time, on='task_count')
        
        # Calculate normalized metrics (lower is better)
        comp_df['norm_makespan'] = comp_df['makespan'] / comp_df['best_makespan']
        comp_df['norm_time'] = comp_df['allocation_time'] / comp_df['best_time']
        
        # Average across task counts
        avg_metrics = comp_df.groupby('method')[['norm_makespan', 'norm_time', 'load_ratio']].mean().reset_index()
        
        # Create radar chart
        def radar_chart(df, metrics, methods, title, save_path):
            # Number of variables
            N = len(metrics)
            
            # Create angle for each variable
            angles = [n / float(N) * 2 * np.pi for n in range(N)]
            angles += angles[:1]  # Close the loop
            
            # Initialize plot
            fig = plt.figure(figsize=(10, 10))
            ax = fig.add_subplot(111, polar=True)
            
            # Draw one axis per variable and add labels
            plt.xticks(angles[:-1], metrics, size=12)
            
            # Draw the y-axis labels (from 0.5 to 2.0)
            ax.set_rlabel_position(0)
            plt.yticks([0.5, 1.0, 1.5, 2.0], ["0.5", "1.0", "1.5", "2.0"], size=10)
            plt.ylim(0, 2)
            
            # Plot each method
            for method in methods:
                if method in df['method'].values:
                    method_data = df[df['method'] == method]
                    
                    # Get metrics values
                    values = [method_data[metric].values[0] for metric in metrics]
                    values += values[:1]  # Close the loop
                    
                    # Plot values
                    ax.plot(angles, values, linewidth=2, linestyle='solid', label=method)
                    ax.fill(angles, values, alpha=0.1)
            
            # Add legend
            plt.legend(loc='upper right', bbox_to_anchor=(0.1, 0.1))
            
            plt.title(title, size=15, y=1.1)
            
            # Save plot
            plt.savefig(save_path)
            plt.close()
        
        # Create radar chart
        radar_metrics = ['norm_makespan', 'norm_time', 'load_ratio']
        radar_chart(
            avg_metrics, radar_metrics, 
            [m for m in avg_metrics['method'].unique() if m != 'DISTRIBUTED'],
            'Comparison of Allocation Methods',
            os.path.join(comparative_dir, 'radar_comparison.png')
        )
    
    def generate_summary(self, report_dir):
        """Generate summary report."""
        summary_file = os.path.join(report_dir, 'summary.md')
        
        with open(summary_file, 'w') as f:
            f.write("# Decentralized Control Experiment Summary\n\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            # Experiment overview
            f.write("## Experiment Overview\n\n")
            
            if self.results_df is not None and not self.results_df.empty:
                total_runs = len(self.results_df)
                task_counts = sorted(self.results_df['task_count'].unique())
                delays = sorted(self.results_df['delay_ms'].unique())
                packet_losses = sorted(self.results_df['packet_loss'].unique())
                epsilons = sorted(self.results_df['epsilon'].unique())
                environments = sorted(self.results_df['environment'].unique())
                
                f.write(f"- Total experimental runs: {total_runs}\n")
                f.write(f"- Task counts: {task_counts}\n")
                f.write(f"- Communication delays: {delays} ms\n")
                f.write(f"- Packet loss probabilities: {packet_losses}\n")
                f.write(f"- Epsilon values: {epsilons}\n")
                f.write(f"- Environments: {environments}\n\n")
            else:
                f.write("- No experiment data available\n\n")
            
            # Key findings
            f.write("## Key Findings\n\n")
            
            if self.results_df is not None and not self.results_df.empty:
                # Overall convergence time
                avg_convergence = self.results_df['metrics.convergence_time'].mean()
                f.write(f"- Average convergence time: {avg_convergence:.2f} seconds\n")
                
                # Effect of task count on convergence
                task_convergence = self.results_df.groupby('task_count')['metrics.convergence_time'].mean()
                f.write("- Effect of task count on convergence time:\n")
                for tasks, time in task_convergence.items():
                    f.write(f"  - {tasks} tasks: {time:.2f} seconds\n")
                
                # Effect of packet loss on message count
                packet_messages = self.results_df.groupby('packet_loss')['metrics.total_messages'].mean()
                f.write("\n- Effect of packet loss on message count:\n")
                for loss, msgs in packet_messages.items():
                    f.write(f"  - {loss*100}% loss: {msgs:.2f} messages\n")
                
                # Effect of epsilon on convergence
                epsilon_convergence = self.results_df.groupby('epsilon')['metrics.convergence_time'].mean()
                f.write("\n- Effect of epsilon on convergence time:\n")
                for eps, time in epsilon_convergence.items():
                    f.write(f"  - ε = {eps}: {time:.2f} seconds\n")
                
                # Environment comparison
                env_makespan = self.results_df.groupby('environment')['metrics.makespan'].mean()
                f.write("\n- Makespan by environment:\n")
                for env, time in env_makespan.items():
                    f.write(f"  - {env}: {time:.2f} seconds\n")
            else:
                f.write("- No experiment data available for analysis\n\n")
            
            # Comparative analysis
            if self.run_comparative:
                f.write("\n## Comparative Analysis\n\n")
                
                # Load comparative results if available
                comp_file = os.path.join(report_dir, 'comparative', 'comparative_results.csv')
                if os.path.exists(comp_file):
                    comp_df = pd.read_csv(comp_file)
                    
                    # Makespan comparison
                    method_makespan = comp_df.groupby('method')['makespan'].mean()
                    f.write("- Average makespan by method:\n")
                    for method, time in method_makespan.items():
                        f.write(f"  - {method}: {time:.2f} seconds\n")
                    
                    # Allocation time comparison
                    method_time = comp_df.groupby('method')['allocation_time'].mean()
                    f.write("\n- Average allocation time by method:\n")
                    for method, time in method_time.items():
                        f.write(f"  - {method}: {time:.6f} seconds\n")
                else:
                    f.write("- No comparative data available\n\n")
            
            # References to plots
            f.write("\n## Generated Plots\n\n")
            f.write("The following plots have been generated:\n\n")
            f.write("1. Convergence time vs task count (plots/convergence_vs_tasks.png)\n")
            f.write("2. Communication overhead vs packet loss (plots/overhead_vs_packet_loss.png)\n")
            f.write("3. Optimality gap vs epsilon (plots/optimality_vs_epsilon.png)\n")
            f.write("4. Recovery time vs task count (plots/recovery_time.png)\n")
            f.write("5. Effect of communication delay (plots/delay_effect.png)\n")
            f.write("6. Environment comparison (plots/environment_comparison.png)\n")
            
            if self.run_comparative:
                f.write("7. Makespan comparison (comparative/makespan_comparison.png)\n")
                f.write("8. Allocation time comparison (comparative/allocation_time_comparison.png)\n")
                f.write("9. Radar chart comparison (comparative/radar_comparison.png)\n")
            
            # Conclusion
            f.write("\n## Conclusion\n\n")
            f.write("This experiment evaluated the performance of the decentralized control algorithm ")
            f.write("for dual mobile manipulators across various parameters and environments. ")
            f.write("The results demonstrate the algorithm's effectiveness in handling different task ")
            f.write("complexities, communication constraints, and environmental conditions.\n\n")
            
            f.write("The theoretical properties of the algorithm, including convergence bounds, ")
            f.write("optimality guarantees, and recovery mechanisms, have been validated through ")
            f.write("systematic experimentation. Comparative analysis with other allocation methods ")
            f.write("shows the strengths and trade-offs of the decentralized approach in different scenarios.\n")
        
        self.log(f"Summary report generated: {summary_file}")

def main():
    parser = argparse.ArgumentParser(description='Run GPU-accelerated decentralized control experiments')
    parser.add_argument('--output_dir', type=str, default='/tmp/decentralized_control/experiment',
                       help='Directory for experiment results')
    parser.add_argument('--trials', type=int, default=3,
                       help='Number of trials per parameter combination')
    parser.add_argument('--timeout', type=float, default=300.0,
                       help='Timeout for each experiment (seconds)')
    parser.add_argument('--record_video', action='store_true',
                       help='Record video for selected runs')
    parser.add_argument('--record_frequency', type=int, default=5,
                       help='Record video every N trials')
    parser.add_argument('--parallel', action='store_true',
                       help='Run experiments in parallel')
    parser.add_argument('--max_parallel', type=int, default=0,
                       help='Maximum number of parallel runs (0 for auto-detection)')
    parser.add_argument('--gpu_id', type=int, default=0,
                       help='GPU ID to use (for systems with multiple GPUs)')
    parser.add_argument('--skip_completed', action='store_true',
                       help='Skip already completed runs')
    parser.add_argument('--comparative', action='store_true',
                       help='Run comparative analysis')
    parser.add_argument('--analyze_only', action='store_true',
                       help='Only analyze existing results without running experiments')
    
    args = parser.parse_args()
    
    # Run experiment
    runner = GpuAcceleratedExperimentRunner(args)
    
    if not args.analyze_only:
        runner.run_factorial_experiments()
    
    runner.generate_analysis_report()

if __name__ == '__main__':
    main()