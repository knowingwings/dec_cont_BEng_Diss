#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import random
import math
import argparse
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from decentralized_auction_mm.msg import Task, TaskArray

class TaskGenerator:
    def __init__(self, config_file=None):
        # Load configuration
        self.config = self.load_config(config_file)
        self.task_id_counter = 1
        
    def load_config(self, config_file):
        # Default configuration
        default_config = {
            'num_tasks': 10,
            'workspace_size': [4.0, 4.0],
            'min_task_spacing': 0.5,
            'dependency_probability': 0.3,
            'max_dependencies_per_task': 3,
            'min_execution_time': 5.0,
            'max_execution_time': 15.0,
            'collaborative_task_ratio': 0.2,
            'capability_patterns': [
                [0.8, 0.4, 0.6, 0.2, 0.3],
                [0.3, 0.9, 0.2, 0.8, 0.4],
                [0.5, 0.5, 0.7, 0.5, 0.9]
            ]
        }
        
        # If config file is provided, load it
        if config_file:
            try:
                with open(config_file, 'r') as f:
                    loaded_config = yaml.safe_load(f)
                    # Update default config with loaded values
                    for key, value in loaded_config.items():
                        if key in default_config:
                            default_config[key] = value
            except Exception as e:
                print(f"Error loading config file: {e}")
        
        return default_config
    
    def generate_tasks(self):
        num_tasks = self.config['num_tasks']
        tasks = []
        positions = self.generate_positions(num_tasks)
        
        # Create tasks
        for i in range(num_tasks):
            task = Task()
            task.id = self.task_id_counter
            self.task_id_counter += 1
            task.name = f"Task_{task.id}"
            
            # Set position
            task.position = self.create_pose(positions[i])
            
            # Set dimensions for visualization
            task.dimensions = Vector3(x=0.1, y=0.1, z=0.1)
            
            # Set execution time
            task.execution_time = random.uniform(
                self.config['min_execution_time'], 
                self.config['max_execution_time']
            )
            
            # Set capability requirements
            pattern_idx = task.id % len(self.config['capability_patterns'])
            base_pattern = self.config['capability_patterns'][pattern_idx]
            task.capabilities_required = self.add_variation(base_pattern)
            
            # Set collaborative flag
            task.is_collaborative = random.random() < self.config['collaborative_task_ratio']
            
            # Initialize other fields
            task.status = Task.STATUS_PENDING
            task.assigned_robot = 0
            task.progress = 0.0
            task.current_price = 0.0
            
            tasks.append(task)
        
        # Add dependencies
        self.add_dependencies(tasks)
        
        return tasks
    
    def generate_positions(self, num_tasks):
        positions = []
        workspace = self.config['workspace_size']
        min_spacing = self.config['min_task_spacing']
        
        for _ in range(num_tasks):
            # Try to find a valid position
            for _ in range(100):  # Max attempts
                x = random.uniform(0.5, workspace[0] - 0.5)
                y = random.uniform(0.5, workspace[1] - 0.5)
                z = 0.8  # Fixed height on tables
                
                # Check distance to existing positions
                valid = True
                for pos in positions:
                    dx = x - pos.x
                    dy = y - pos.y
                    dist = math.sqrt(dx*dx + dy*dy)
                    if dist < min_spacing:
                        valid = False
                        break
                
                if valid:
                    positions.append(Point(x=x, y=y, z=z))
                    break
        
        return positions
    
    def create_pose(self, point):
        pose = Pose()
        pose.position = point
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return pose
    
    def add_variation(self, base_pattern):
        varied = []
        for val in base_pattern:
            # Add small random variation
            variation = random.gauss(0, 0.1)
            varied.append(max(0.1, min(1.0, val + variation)))
        return varied
    
    def add_dependencies(self, tasks):
        if len(tasks) <= 1:
            return
        
        # Sort tasks by ID to ensure no cycles
        tasks.sort(key=lambda t: t.id)
        
        for i in range(1, len(tasks)):
            # Each task can depend on tasks with lower IDs
            for j in range(i):
                # Add dependency with specified probability
                if random.random() < self.config['dependency_probability']:
                    # Limit maximum number of dependencies
                    if len(tasks[i].prerequisites) < self.config['max_dependencies_per_task']:
                        tasks[i].prerequisites.append(tasks[j].id)
    
    def save_tasks_to_file(self, tasks, output_file):
        # Create a TaskArray message
        task_array = TaskArray()
        task_array.tasks = tasks
        
        # Convert to YAML representation for storage
        task_data = []
        for task in tasks:
            task_dict = {
                'id': task.id,
                'name': task.name,
                'position': {
                    'x': task.position.position.x,
                    'y': task.position.position.y,
                    'z': task.position.position.z
                },
                'execution_time': task.execution_time,
                'is_collaborative': task.is_collaborative,
                'prerequisites': list(task.prerequisites),
                'capabilities_required': list(task.capabilities_required)
            }
            task_data.append(task_dict)
        
        with open(output_file, 'w') as f:
            yaml.dump(task_data, f)
        
        print(f"Saved {len(tasks)} tasks to {output_file}")

def main():
    parser = argparse.ArgumentParser(description='Generate tasks for decentralized auction system')
    parser.add_argument('--config', type=str, help='Path to configuration file')
    parser.add_argument('--output', type=str, default='tasks.yaml', help='Output file path')
    parser.add_argument('--num-tasks', type=int, help='Number of tasks to generate')
    args = parser.parse_args()
    
    generator = TaskGenerator(args.config)
    
    # Override num_tasks if specified
    if args.num_tasks:
        generator.config['num_tasks'] = args.num_tasks
    
    tasks = generator.generate_tasks()
    generator.save_tasks_to_file(tasks, args.output)
    
    print(f"Generated {len(tasks)} tasks")
    
if __name__ == '__main__':
    main()