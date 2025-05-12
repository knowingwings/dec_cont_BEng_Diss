# assembly_task_visualizer.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose, Point, Quaternion
import random
import string
from builtin_interfaces.msg import Duration

from decentralized_control.msg import Task, TaskList, TaskAssignment

class AssemblyTaskVisualizer(Node):
    """
    Visualizes assembly tasks in Gazebo by spawning models and markers.
    """
    
    def __init__(self):
        super().__init__('assembly_task_visualizer')
        
        # Initialize task storage
        self.tasks = {}  # task_id -> Task
        self.task_models = {}  # task_id -> model_name
        self.task_assignments = {}  # task_id -> robot_id
        self.task_completion = {}  # task_id -> completion status
        
        # Setup service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Wait for services to be available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn_entity service...')
        
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for delete_entity service...')
        
        # Subscribers
        self.task_subscriber = self.create_subscription(
            TaskList, '/tasks', self.task_callback, 10)
        
        self.assignment_subscriber = self.create_subscription(
            TaskAssignment, '/auction/assignments', self.assignment_callback, 10)
        
        # Timer for updating visualizations
        self.visualization_timer = self.create_timer(0.5, self.update_visualizations)
        
        self.get_logger().info('Assembly task visualizer initialized')
    
    def task_callback(self, msg):
        """Process incoming tasks and visualize them."""
        for task in msg.tasks:
            # Store task
            self.tasks[task.id] = task
            self.task_completion[task.id] = False
            
            # Generate unique model name for this task
            if task.id not in self.task_models:
                model_name = f'task_{task.id}_{self.generate_random_suffix()}'
                self.task_models[task.id] = model_name
                
                # Spawn visual representation
                self.spawn_task_model(task, model_name)
    
    def assignment_callback(self, msg):
        """Process task assignments and update visualizations."""
        for i in range(len(msg.task_ids)):
            task_id = msg.task_ids[i]
            robot_id = msg.robot_ids[i]
            
            # Update assignment
            self.task_assignments[task_id] = robot_id
    
    def generate_random_suffix(self, length=5):
        """Generate random alphanumeric suffix for unique model names."""
        return ''.join(random.choice(string.ascii_lowercase + string.digits) for _ in range(length))
    
    def spawn_task_model(self, task, model_name):
        """Spawn a visual representation of a task in Gazebo."""
        # Model XML (color based on task type/requirements)
        capability_sum = sum(task.capabilities_required)
        
        # Normalize to [0, 1] range and use for coloring
        r = 0.2 + 0.8 * (task.capabilities_required[0] / capability_sum if capability_sum > 0 else 0.5)
        g = 0.2 + 0.8 * (task.capabilities_required[1] / capability_sum if capability_sum > 0 else 0.5)
        b = 0.2 + 0.8 * (task.capabilities_required[2] / capability_sum if capability_sum > 0 else 0.5)
        
        # Scale size based on execution time
        size_scale = 0.05 + 0.02 * min(5, task.execution_time) / 5.0
        
        # Different shape for collaborative tasks
        if hasattr(task, 'requires_collaboration') and task.requires_collaboration:
            shape_xml = f"""
            <cylinder>
              <radius>{size_scale}</radius>
              <length>0.02</length>
            </cylinder>
            """
        else:
            shape_xml = f"""
            <box>
              <size>{size_scale} {size_scale} 0.02</size>
            </box>
            """
        
        # Generate model XML
        model_xml = f"""
        <sdf version='1.6'>
          <model name='{model_name}'>
            <static>true</static>
            <link name='link'>
              <visual name='visual'>
                <geometry>
                  {shape_xml}
                </geometry>
                <material>
                  <ambient>{r} {g} {b} 0.7</ambient>
                  <diffuse>{r} {g} {b} 0.7</diffuse>
                  <specular>0.1 0.1 0.1 0.7</specular>
                  <emissive>0 0 0 0</emissive>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """
        
        # Spawn the model
        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = model_xml
        request.robot_namespace = ''
        request.initial_pose = Pose()
        request.initial_pose.position.x = task.position[0]
        request.initial_pose.position.y = task.position[1]
        request.initial_pose.position.z = task.position[2] + 0.01  # Slightly above surface
        
        # Call spawn service
        future = self.spawn_client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'Task {task.id} visualization spawned'))
    
    def update_visualizations(self):
        """Update task visualizations based on assignments and completions."""
        for task_id, model_name in list(self.task_models.items()):
            # Skip if task is completed and model is already removed
            if task_id in self.task_completion and self.task_completion[task_id]:
                continue
            
            # Check if task is assigned
            if task_id in self.task_assignments:
                robot_id = self.task_assignments[task_id]
                
                # If task is assigned, update its visualization
                if robot_id > 0:
                    # In a real implementation, would modify the model color or add a marker
                    # For simplicity, just log the assignment
                    self.get_logger().debug(f'Task {task_id} visualized as assigned to Robot {robot_id}')
                
                # If task is completed (simulated), remove it
                # In a real implementation, this would come from task status updates
                # Here we simulate completion after random time
                if robot_id > 0 and random.random() < 0.001:  # Small chance of completion each update
                    self.remove_task_model(task_id, model_name)
                    self.task_completion[task_id] = True
    
    def remove_task_model(self, task_id, model_name):
        """Remove the model for a completed task."""
        request = DeleteEntity.Request()
        request.name = model_name
        
        future = self.delete_client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'Task {task_id} visualization removed (completed)'))