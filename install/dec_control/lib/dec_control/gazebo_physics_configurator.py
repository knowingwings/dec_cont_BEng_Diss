# gazebo_physics_configurator.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.msg import ODEPhysics, ContactsState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

class GazeboPhysicsConfigurator(Node):
    """
    Configures Gazebo physics properties for realistic manipulation.
    """
    
    def __init__(self):
        super().__init__('gazebo_physics_configurator')
        
        # Physics parameters
        self.declare_parameter('max_step_size', 0.001)
        self.declare_parameter('real_time_factor', 1.0)
        self.declare_parameter('real_time_update_rate', 1000.0)
        self.declare_parameter('gravity_x', 0.0)
        self.declare_parameter('gravity_y', 0.0)
        self.declare_parameter('gravity_z', -9.8)
        self.declare_parameter('ode_solver_type', 'quick')
        self.declare_parameter('ode_min_step_size', 0.0001)
        self.declare_parameter('ode_iters', 50)
        self.declare_parameter('ode_sor', 1.3)
        self.declare_parameter('ode_friction_model', 'pyramid_model')
        self.declare_parameter('contact_max_correcting_vel', 100.0)
        self.declare_parameter('contact_surface_layer', 0.001)
        self.declare_parameter('max_contacts', 20)
        
        # Get parameters
        self.max_step_size = self.get_parameter('max_step_size').value
        self.real_time_factor = self.get_parameter('real_time_factor').value
        self.real_time_update_rate = self.get_parameter('real_time_update_rate').value
        self.gravity_x = self.get_parameter('gravity_x').value
        self.gravity_y = self.get_parameter('gravity_y').value
        self.gravity_z = self.get_parameter('gravity_z').value
        self.ode_solver_type = self.get_parameter('ode_solver_type').value
        self.ode_min_step_size = self.get_parameter('ode_min_step_size').value
        self.ode_iters = self.get_parameter('ode_iters').value
        self.ode_sor = self.get_parameter('ode_sor').value
        self.ode_friction_model = self.get_parameter('ode_friction_model').value
        self.contact_max_correcting_vel = self.get_parameter('contact_max_correcting_vel').value
        self.contact_surface_layer = self.get_parameter('contact_surface_layer').value
        self.max_contacts = self.get_parameter('max_contacts').value
        
        # Setup client for physics service
        self.physics_client = self.create_client(SetPhysicsProperties, '/set_physics_properties')
        
        # Wait for service to be available
        while not self.physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_physics_properties service...')
        
        # Timer to set physics properties after a short delay
        self.physics_timer = self.create_timer(2.0, self.set_physics_properties)
        
        # Contact monitoring for force feedback
        self.contact_subscribers = []
        for i in range(1, 3):  # For dual robots
            self.contact_subscribers.append(
                self.create_subscription(
                    ContactsState, f'/robot{i}/gripper_contacts',
                    self.contact_callback, 10))
        
        self.get_logger().info('Gazebo physics configurator initialized')
    
    def set_physics_properties(self):
        """Set custom physics properties in Gazebo."""
        request = SetPhysicsProperties.Request()
        
        # Time step settings
        request.time_step = Float64(data=self.max_step_size)
        request.max_update_rate = Float64(data=self.real_time_update_rate)
        
        # Gravity vector
        request.gravity = Vector3()
        request.gravity.x = self.gravity_x
        request.gravity.y = self.gravity_y
        request.gravity.z = self.gravity_z
        
        # ODE Parameters
        request.ode_config = ODEPhysics()
        request.ode_config.auto_disable_bodies = False
        request.ode_config.sor_pgs_precon_iters = 0
        request.ode_config.sor_pgs_iters = self.ode_iters
        request.ode_config.sor_pgs_w = self.ode_sor
        request.ode_config.sor_pgs_rms_error_tol = 0.0
        request.ode_config.contact_surface_layer = self.contact_surface_layer
        request.ode_config.contact_max_correcting_vel = self.contact_max_correcting_vel
        request.ode_config.cfm = 0.0
        request.ode_config.erp = 0.2
        request.ode_config.max_contacts = self.max_contacts
        
        # Send physics properties request
        future = self.physics_client.call_async(request)
        future.add_done_callback(self.physics_callback)
        
        # Cancel timer to prevent multiple calls
        self.physics_timer.cancel()
    
    def physics_callback(self, future):
        """Callback for physics service response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Physics properties set successfully')
            else:
                self.get_logger().error(
                    f'Failed to set physics properties: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def contact_callback(self, msg):
        """Process contact information for force feedback."""
        if not msg.states:
            return
        
        # Calculate average contact force
        total_force = Vector3()
        for state in msg.states:
            # Accumulate forces
            for force in state.wrenches:
                total_force.x += force.force.x
                total_force.y += force.force.y
                total_force.z += force.force.z
        
        # Average force
        avg_force = Vector3()
        if len(msg.states) > 0:
            avg_force.x = total_force.x / len(msg.states)
            avg_force.y = total_force.y / len(msg.states)
            avg_force.z = total_force.z / len(msg.states)
        
        # Log significant contact forces
        force_magnitude = (avg_force.x**2 + avg_force.y**2 + avg_force.z**2)**0.5
        if force_magnitude > 5.0:  # Threshold for significant contact
            self.get_logger().info(f'Significant contact detected: {force_magnitude:.2f} N')