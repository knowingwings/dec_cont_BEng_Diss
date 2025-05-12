# robot_model_spawner.py
#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import xacro
from ament_index_python.packages import get_package_share_directory

class RobotModelSpawner(Node):
    """
    Configures and spawns robot models with manipulators in Gazebo.
    """
    
    def __init__(self):
        super().__init__('robot_model_spawner')
        
        # Parameters
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('robot_model', 'waffle_pi')
        self.declare_parameter('manipulator_model', 'open_manipulator_x')
        self.declare_parameter('position', [0.0, 0.0, 0.0])
        self.declare_parameter('orientation', [0.0, 0.0, 0.0])
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_model = self.get_parameter('robot_model').value
        self.manipulator_model = self.get_parameter('manipulator_model').value
        self.position = self.get_parameter('position').value
        self.orientation = self.get_parameter('orientation').value
        
        # Setup client for spawn service
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # Wait for service to be available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn_entity service...')
        
        # Generate combined URDF for mobile manipulator
        self.combined_urdf = self.generate_combined_urdf()
        
        # Timer to spawn robot after a short delay
        self.spawn_timer = self.create_timer(1.0, self.spawn_robot)
        
        self.get_logger().info(
            f'Robot model spawner initialized for Robot {self.robot_id}')
    
    def generate_combined_urdf(self):
        """Generate combined URDF for mobile base and manipulator."""
        # Get package paths
        turtlebot_pkg = get_package_share_directory('turtlebot3_description')
        manipulator_pkg = get_package_share_directory('open_manipulator_x_description')
        
        # Create a temporary combined XACRO file
        combined_xacro_path = os.path.join(
            get_package_share_directory('decentralized_control'),
            'urdf', f'mobile_manipulator_{self.robot_id}.urdf.xacro')
        
        # Ensure directory exists
        os.makedirs(os.path.dirname(combined_xacro_path), exist_ok=True)
        
        # Create combined XACRO content
        combined_content = f"""<?xml version="1.0"?>
<robot name="mobile_manipulator_{self.robot_id}" xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- Include TurtleBot3 model -->
  <xacro:include filename="{turtlebot_pkg}/urdf/turtlebot3_{self.robot_model}.urdf.xacro" />
  
  <!-- Include OpenMANIPULATOR-X model -->
  <xacro:include filename="{manipulator_pkg}/urdf/open_manipulator_x.urdf.xacro" />
  
  <!-- Mount manipulator on top of TurtleBot -->
  <joint name="manipulator_base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>
  
  <!-- Additional gazebo plugins for controller integration -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>robot{self.robot_id}</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
      <legacyModeNS>true</legacyModeNS>
    </plugin>
  </gazebo>
</robot>
"""
        
        # Write combined XACRO file
        with open(combined_xacro_path, 'w') as f:
            f.write(combined_content)
        
        # Process XACRO file to generate URDF
        urdf_content = xacro.process_file(combined_xacro_path).toxml()
        
        return urdf_content
    
    def spawn_robot(self):
        """Spawn robot model in Gazebo."""
        request = SpawnEntity.Request()
        request.name = f'robot{self.robot_id}'
        request.xml = self.combined_urdf
        request.robot_namespace = f'robot{self.robot_id}'
        request.initial_pose.position.x = self.position[0]
        request.initial_pose.position.y = self.position[1]
        request.initial_pose.position.z = self.position[2]
        
        # Convert Euler angles to quaternion (simplified)
        request.initial_pose.orientation.w = 1.0
        
        # Send spawn request
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)
        
        # Cancel timer to prevent multiple spawns
        self.spawn_timer.cancel()
    
    def spawn_callback(self, future):
        """Callback for spawn service response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'Robot {self.robot_id} spawned successfully')
                
                # Set up controllers for the manipulator
                self.setup_controllers()
            else:
                self.get_logger().error(
                    f'Failed to spawn robot {self.robot_id}: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def setup_controllers(self):
        """Set up controllers for the manipulator."""
        # In a real implementation, would call services to load and configure controllers
        # For demonstration, just log the intention
        self.get_logger().info(
            f'Setting up controllers for robot {self.robot_id} manipulator')