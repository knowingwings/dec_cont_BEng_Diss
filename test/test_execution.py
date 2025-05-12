#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
import time
import numpy as np
from threading import Thread

from decentralized_control.execution.execution_controller import ExecutionController
from decentralized_control.msg import Task, TaskList, TaskAssignment, RobotState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class RobotStateSubscriber(Node):
    """Test node for subscribing to robot state."""
    
    def __init__(self, robot_id):
        super().__init__(f'robot_state_subscriber{robot_id}')
        self.subscription = self.create_subscription(
            RobotState, f'/robot{robot_id}/state', self.state_callback, 10)
        self.latest_state = None
        self.update_count = 0
    
    def state_callback(self, msg):
        self.latest_state = msg
        self.update_count += 1

class TestExecution(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.execution_controller = ExecutionController()
        
        # Override parameters
        cls.execution_controller.robot_id = 1
        
        # Create publisher nodes
        cls.odom_pub = Node('odom_publisher')
        cls.joint_state_pub = Node('joint_state_publisher')
        cls.task_assign_pub = Node('task_assign_publisher')
        
        # Create publishers
        cls.odom_publisher = cls.odom_pub.create_publisher(
            Odometry, f'/robot{cls.execution_controller.robot_id}/odom', 10)
        cls.joint_state_publisher = cls.joint_state_pub.create_publisher(
            JointState, f'/robot{cls.execution_controller.robot_id}/joint_states', 10)
        cls.task_assign_publisher = cls.task_assign_pub.create_publisher(
            TaskAssignment, '/auction/assignments', 10)
        
        # Create subscriber nodes
        cls.state_subscriber = RobotStateSubscriber(cls.execution_controller.robot_id)
        cls.cmd_vel_sub = Node('cmd_vel_subscriber')
        cls.joint_cmd_sub = Node('joint_cmd_subscriber')
        
        # Create subscribers
        cls.cmd_vel_subscription = cls.cmd_vel_sub.create_subscription(
            Twist, f'/robot{cls.execution_controller.robot_id}/cmd_vel',
            lambda msg: setattr(cls, 'latest_cmd_vel', msg), 10)
        cls.joint_cmd_subscription = cls.joint_cmd_sub.create_subscription(
            Float64MultiArray, f'/robot{cls.execution_controller.robot_id}/arm_controller/command',
            lambda msg: setattr(cls, 'latest_joint_cmd', msg), 10)
        
        # Initialize latest message variables
        cls.latest_cmd_vel = None
        cls.latest_joint_cmd = None
        
        # Start spinning in separate threads
        cls.controller_thread = Thread(target=lambda: rclpy.spin(cls.execution_controller))
        cls.odom_thread = Thread(target=lambda: rclpy.spin(cls.odom_pub))
        cls.joint_thread = Thread(target=lambda: rclpy.spin(cls.joint_state_pub))
        cls.task_thread = Thread(target=lambda: rclpy.spin(cls.task_assign_pub))
        cls.state_thread = Thread(target=lambda: rclpy.spin(cls.state_subscriber))
        cls.cmd_vel_thread = Thread(target=lambda: rclpy.spin(cls.cmd_vel_sub))
        cls.joint_cmd_thread = Thread(target=lambda: rclpy.spin(cls.joint_cmd_sub))
        
        cls.controller_thread.daemon = True
        cls.odom_thread.daemon = True
        cls.joint_thread.daemon = True
        cls.task_thread.daemon = True
        cls.state_thread.daemon = True
        cls.cmd_vel_thread.daemon = True
        cls.joint_cmd_thread.daemon = True
        
        cls.controller_thread.start()
        cls.odom_thread.start()
        cls.joint_thread.start()
        cls.task_thread.start()
        cls.state_thread.start()
        cls.cmd_vel_thread.start()
        cls.joint_cmd_thread.start()
        
        # Wait for nodes to initialize
        time.sleep(1)
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def test_odometry_update(self):
        """Test that odometry updates robot position."""
        # Send odometry message
        odom = Odometry()
        odom.header.stamp = self.execution_controller.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        
        # Set position
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.position.y = 2.0
        odom.pose.pose.position.z = 0.0
        
        # Set orientation (simplified quaternion for 90 degrees yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.7071
        odom.pose.pose.orientation.w = 0.7071
        
        self.odom_publisher.publish(odom)
        
        # Wait for odometry to be processed
        time.sleep(0.1)
        
        # Verify position update
        self.assertAlmostEqual(self.execution_controller.position[0], 1.0, places=1)
        self.assertAlmostEqual(self.execution_controller.position[1], 2.0, places=1)
        
        # Verify orientation update (should be approximately 90 degrees / PI/2)
        self.assertAlmostEqual(self.execution_controller.orientation[2], np.pi/2, places=1)
    
    def test_joint_state_update(self):
        """Test that joint states update controller state."""
        # Send joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.execution_controller.get_clock().now().to_msg()
        joint_state.name = ["joint1", "joint2", "joint3", "joint4", "gripper"]
        joint_state.position = [0.1, 0.2, 0.3, 0.4, 0.5]
        
        self.joint_state_publisher.publish(joint_state)
        
        # Wait for joint state to be processed
        time.sleep(0.1)
        
        # Verify joint position update
        for i in range(5):
            self.assertAlmostEqual(self.execution_controller.joint_positions[i], i*0.1 + 0.1, places=1)
    
    def test_task_assignment(self):
        """Test that task assignments are processed."""
        # Create task assignment
        assignment = TaskAssignment()
        assignment.task_ids = [1, 2, 3]
        assignment.robot_ids = [1, 2, 1]  # Tasks 1 and 3 assigned to robot 1
        assignment.prices = [0.0, 0.0, 0.0]
        
        self.task_assign_publisher.publish(assignment)
        
        # Wait for assignment to be processed
        time.sleep(0.1)
        
        # Verify assigned tasks
        with self.execution_controller.task_lock:
            self.assertEqual(len(self.execution_controller.assigned_tasks), 2)
            
            # Check task IDs
            task_ids = [task.id for task in self.execution_controller.assigned_tasks]
            self.assertIn(1, task_ids)
            self.assertIn(3, task_ids)
            self.assertNotIn(2, task_ids)
    
    def test_state_publishing(self):
        """Test that robot state is published correctly."""
        # Set up controller state
        self.execution_controller.position = np.array([1.0, 2.0, 3.0])
        self.execution_controller.orientation = np.array([0.1, 0.2, 0.3])
        
        # Add some tasks to the controller
        with self.execution_controller.task_lock:
            task1 = Task()
            task1.id = 1
            task1.execution_time = 5.0
            
            task2 = Task()
            task2.id = 2
            task2.execution_time = 3.0
            
            self.execution_controller.assigned_tasks = [task1, task2]
        
        # Force state publishing
        self.execution_controller.publish_state()
        
        # Wait for state to be published
        time.sleep(0.1)
        
        # Verify state was published
        self.assertIsNotNone(self.state_subscriber.latest_state)
        self.assertGreater(self.state_subscriber.update_count, 0)
        
        # Verify state content
        state = self.state_subscriber.latest_state
        self.assertEqual(state.id, 1)
        self.assertAlmostEqual(state.position[0], 1.0)
        self.assertAlmostEqual(state.position[1], 2.0)
        self.assertAlmostEqual(state.position[2], 3.0)
        self.assertAlmostEqual(state.orientation[0], 0.1)
        self.assertAlmostEqual(state.orientation[1], 0.2)
        self.assertAlmostEqual(state.orientation[2], 0.3)
        
        # Verify workload calculation
        self.assertEqual(state.workload, 8.0)  # 5.0 + 3.0
        self.assertFalse(state.failed)
    
    def test_task_execution(self):
        """Test that tasks are executed in order."""
        # Clear current tasks
        with self.execution_controller.task_lock:
            self.execution_controller.assigned_tasks = []
            self.execution_controller.current_task = None
            self.execution_controller.executing = False
        
        # Create test task
        task = Task()
        task.id = 5
        task.position = [1.0, 1.0, 0.0]
        task.capabilities_required = [1.0, 1.0, 1.0, 1.0, 1.0]
        task.execution_time = 0.5  # Short execution time for testing
        
        # Add task to assignment
        assignment = TaskAssignment()
        assignment.task_ids = [5]
        assignment.robot_ids = [1]
        assignment.prices = [0.0]
        
        self.task_assign_publisher.publish(assignment)
        
        # Wait for task to be assigned
        time.sleep(0.2)
        
        # Verify task is being executed
        self.assertIsNotNone(self.execution_controller.current_task)
        self.assertEqual(self.execution_controller.current_task.id, 5)
        self.assertTrue(self.execution_controller.executing)
        
        # Get initial progress
        initial_progress = self.execution_controller.task_progress
        
        # Wait for progress to increase
        time.sleep(0.2)
        
        # Verify progress has increased
        self.assertGreater(self.execution_controller.task_progress, initial_progress)
        
        # Wait for task to complete
        time.sleep(0.5)
        
        # Verify task completed
        self.assertFalse(self.execution_controller.executing)
        self.assertIsNone(self.execution_controller.current_task)

if __name__ == '__main__':
    unittest.main()