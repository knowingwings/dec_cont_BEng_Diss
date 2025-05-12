# industrial_scenarios.py
#!/usr/bin/env python3

import csv
import numpy as np
import os
import yaml
import rclpy
from rclpy.node import Node
from decentralized_control.msg import Task, TaskList

class IndustrialScenarioGenerator(Node):
    """
    Generates realistic industrial assembly scenarios based on 
    RobMoSys ITP EU Project data and NIST assembly benchmark datasets.
    """
    
    def __init__(self):
        super().__init__('industrial_scenario_generator')
        
        # Parameters
        self.declare_parameter('scenario_type', 'automotive')  # automotive, electronics, or furniture
        self.declare_parameter('complexity', 'medium')  # low, medium, high
        self.declare_parameter('environment_scale', 1.0)  # scaling factor for environment
        self.declare_parameter('output_dir', '/tmp/decentralized_control/scenarios')
        
        # Get parameters
        self.scenario_type = self.get_parameter('scenario_type').value
        self.complexity = self.get_parameter('complexity').value
        self.scale_factor = self.get_parameter('environment_scale').value
        self.output_dir = self.get_parameter('output_dir').value
        
        # Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Publishers
        self.task_publisher = self.create_publisher(TaskList, '/tasks', 10)
        
        # Reference datasets
        self.datasets = {
            'automotive': {
                'source': 'NIST Assembly Task Board for Robotic Assembly',
                'citation': 'Kimble et al. (2020). The NIST Assembly Task Board for Robotic Assembly. Transactions on Industrial Informatics.',
                'url': 'https://www.nist.gov/el/intelligent-systems-division-73500/robotic-grasping-and-manipulation-assembly',
                'task_count': {'low': 8, 'medium': 16, 'high': 32},
                'dependency_density': {'low': 0.1, 'medium': 0.3, 'high': 0.5},
                'precision_requirements': {'low': 1.0, 'medium': 0.5, 'high': 0.2}  # mm
            },
            'electronics': {
                'source': 'World Robot Summit Assembly Challenge',
                'citation': 'Kotani et al. (2021). WRS Assembly Challenge: A Realistic Benchmark for Robotic Assembly. IEEE Robotics and Automation Letters.',
                'url': 'https://wrs.nedo.go.jp/en/industrial/assembly/index.html',
                'task_count': {'low': 10, 'medium': 20, 'high': 40},
                'dependency_density': {'low': 0.2, 'medium': 0.4, 'high': 0.6},
                'precision_requirements': {'low': 0.5, 'medium': 0.2, 'high': 0.1}  # mm
            },
            'furniture': {
                'source': 'RobMoSys ITP EU Project',
                'citation': 'Lutz et al. (2021). Experience in System Design and Validation using the RobMoSys Ecosystem.',
                'url': 'https://robmosys.eu/results-itp/',
                'task_count': {'low': 6, 'medium': 12, 'high': 24},
                'dependency_density': {'low': 0.3, 'medium': 0.5, 'high': 0.7},
                'precision_requirements': {'low': 2.0, 'medium': 1.0, 'high': 0.5}  # mm
            }
        }
        
        # Create timer for generating scenario
        self.timer = self.create_timer(1.0, self.generate_scenario)
        self.scenario_generated = False
        
        self.get_logger().info(
            f'Industrial scenario generator initialized for {self.scenario_type} assembly, '
            f'{self.complexity} complexity')
    
    def generate_scenario(self):
        """Generate industrial assembly scenario based on selected parameters."""
        if self.scenario_generated:
            self.timer.cancel()
            return
        
        # Get dataset parameters
        dataset = self.datasets.get(self.scenario_type, self.datasets['automotive'])
        task_count = dataset['task_count'][self.complexity]
        dependency_density = dataset['dependency_density'][self.complexity]
        precision_req = dataset['precision_requirements'][self.complexity]
        
        # Create tasks
        tasks = []
        
        # Task types specific to scenario
        if self.scenario_type == 'automotive':
            task_types = [
                {'name': 'insert_fastener', 'execution_time': 3.0, 'capabilities': [0.8, 0.7, 0.9, 0.4, 0.3]},
                {'name': 'attach_panel', 'execution_time': 8.0, 'capabilities': [0.9, 0.5, 0.4, 0.8, 0.7]},
                {'name': 'align_component', 'execution_time': 5.0, 'capabilities': [0.6, 0.9, 0.7, 0.5, 0.4]},
                {'name': 'apply_sealant', 'execution_time': 4.0, 'capabilities': [0.5, 0.6, 0.8, 0.7, 0.9]},
                {'name': 'tighten_bolt', 'execution_time': 2.0, 'capabilities': [0.7, 0.4, 0.6, 0.9, 0.5]}
            ]
            # Based on NIST Assembly Task Board workflow
            station_positions = [
                [1.0, 1.0, 0.5],  # Fastener station
                [1.0, 2.0, 0.5],  # Panel station
                [2.0, 1.0, 0.5],  # Alignment station
                [2.0, 2.0, 0.5],  # Sealant station
                [3.0, 1.5, 0.5]   # Assembly station
            ]
            collaborative_ratio = 0.2  # 20% of tasks require collaboration
            
        elif self.scenario_type == 'electronics':
            task_types = [
                {'name': 'pick_component', 'execution_time': 2.0, 'capabilities': [0.7, 0.9, 0.8, 0.3, 0.5]},
                {'name': 'place_pcb', 'execution_time': 4.0, 'capabilities': [0.9, 0.8, 0.6, 0.7, 0.4]},
                {'name': 'solder_joint', 'execution_time': 6.0, 'capabilities': [0.8, 0.5, 0.9, 0.6, 0.7]},
                {'name': 'inspect_connection', 'execution_time': 3.0, 'capabilities': [0.4, 0.6, 0.5, 0.9, 0.8]},
                {'name': 'test_circuit', 'execution_time': 5.0, 'capabilities': [0.5, 0.4, 0.7, 0.8, 0.9]}
            ]
            # Based on WRS Assembly Challenge layout
            station_positions = [
                [1.0, 0.5, 0.5],  # Component tray
                [1.0, 1.5, 0.5],  # PCB holder
                [2.0, 1.0, 0.5],  # Soldering station
                [2.5, 2.0, 0.5],  # Inspection station
                [3.0, 1.0, 0.5]   # Testing station
            ]
            collaborative_ratio = 0.3  # 30% of tasks require collaboration
            
        else:  # furniture
            task_types = [
                {'name': 'align_panels', 'execution_time': 5.0, 'capabilities': [0.9, 0.8, 0.7, 0.5, 0.3]},
                {'name': 'insert_dowel', 'execution_time': 3.0, 'capabilities': [0.7, 0.9, 0.8, 0.4, 0.5]},
                {'name': 'apply_glue', 'execution_time': 2.0, 'capabilities': [0.6, 0.7, 0.9, 0.5, 0.8]},
                {'name': 'tighten_screw', 'execution_time': 4.0, 'capabilities': [0.8, 0.5, 0.6, 0.9, 0.4]},
                {'name': 'rotate_assembly', 'execution_time': 7.0, 'capabilities': [0.9, 0.6, 0.5, 0.7, 0.8]}
            ]
            # Based on RobMoSys furniture assembly setup
            station_positions = [
                [0.5, 1.0, 0.5],  # Panel storage
                [1.5, 1.0, 0.5],  # Dowel insertion
                [2.0, 0.5, 0.5],  # Glue station
                [2.0, 1.5, 0.5],  # Screw station
                [3.0, 1.0, 0.5]   # Assembly area
            ]
            collaborative_ratio = 0.4  # 40% of tasks require collaboration
        
        # Scale positions based on environment scale factor
        scaled_positions = [[x * self.scale_factor for x in pos] for pos in station_positions]
        
        # Generate tasks
        for i in range(1, task_count + 1):
            task = Task()
            task.id = i
            
            # Determine task type
            task_type = task_types[i % len(task_types)]
            
            # Determine task position (at appropriate station)
            station_idx = i % len(scaled_positions)
            base_pos = scaled_positions[station_idx]
            
            # Add small random offset for realistic positioning
            position = [
                base_pos[0] + 0.1 * (np.random.random() - 0.5),
                base_pos[1] + 0.1 * (np.random.random() - 0.5),
                base_pos[2] + 0.05 * (np.random.random() - 0.5)
            ]
            task.position = position
            
            # Set capabilities required
            task.capabilities_required = task_type['capabilities']
            
            # Set execution time (with small random variation)
            base_time = task_type['execution_time']
            task.execution_time = base_time * (0.9 + 0.2 * np.random.random())
            
            # Initialize with no prerequisites (will be added later)
            task.prerequisites = []
            
            # Determine if collaborative
            task.requires_collaboration = np.random.random() < collaborative_ratio
            
            tasks.append(task)
        
        # Add dependencies based on density and realistic assembly constraints
        for i in range(1, task_count):
            task = tasks[i]
            
            # For realistic assembly, each task has higher probability to depend
            # on temporally adjacent tasks (mimicking assembly sequence)
            for j in range(i):
                prev_task = tasks[j]
                
                # Probability decreases with temporal distance
                prob = dependency_density * (1.0 - 0.5 * (i - j) / i)
                
                if np.random.random() < prob:
                    task.prerequisites.append(prev_task.id)
            
            # Limit maximum prerequisites to keep problem tractable
            if len(task.prerequisites) > 3:
                # Keep only 3 random prerequisites
                task.prerequisites = np.random.choice(
                    task.prerequisites, size=3, replace=False).tolist()
        
        # Create task list message
        task_list = TaskList()
        task_list.tasks = tasks
        
        # Publish task list
        self.task_publisher.publish(task_list)
        
        # Save scenario definition to YAML file
        scenario_data = {
            'metadata': {
                'scenario_type': self.scenario_type,
                'complexity': self.complexity,
                'source': dataset['source'],
                'citation': dataset['citation'],
                'url': dataset['url'],
                'precision_requirements_mm': precision_req
            },
            'tasks': []
        }
        
        for task in tasks:
            task_data = {
                'id': task.id,
                'position': task.position,
                'execution_time': task.execution_time,
                'prerequisites': task.prerequisites,
                'requires_collaboration': task.requires_collaboration,
                'capabilities_required': task.capabilities_required
            }
            scenario_data['tasks'].append(task_data)
        
        # Write to file
        filename = os.path.join(
            self.output_dir, 
            f'industrial_scenario_{self.scenario_type}_{self.complexity}.yaml')
        
        with open(filename, 'w') as f:
            yaml.dump(scenario_data, f, default_flow_style=False)
        
        self.get_logger().info(
            f'Generated {task_count} tasks for {self.scenario_type} assembly scenario. '
            f'Saved to {filename}')
        
        # Mark as generated to avoid duplicates
        self.scenario_generated = True