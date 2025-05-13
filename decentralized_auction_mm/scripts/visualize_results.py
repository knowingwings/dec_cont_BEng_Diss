#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA

from decentralized_auction_mm.msg import Task, TaskArray, Bid, BidArray, RobotStatus, AuctionStatus

class AuctionVisualizer(Node):
    """
    Visualizer for the distributed auction algorithm
    
    This node subscribes to auction data and visualizes:
    - Tasks and their assignments
    - Robot positions and status
    - Bids and auction progress
    - Task dependencies as arrows
    """
    
    def __init__(self):
        super().__init__('auction_visualizer')
        
        # Create subscribers
        self.task_sub = self.create_subscription(
            TaskArray, 'tasks', self.task_callback, 10)
        self.robot_status_sub = self.create_subscription(
            RobotStatus, 'robot_status', self.robot_status_callback, 10)
        self.bid_sub = self.create_subscription(
            Bid, 'bids', self.bid_callback, 10)
        self.auction_status_sub = self.create_subscription(
            AuctionStatus, 'auction_status', self.auction_status_callback, 10)
        
        # Create publishers
        self.task_markers_pub = self.create_publisher(
            MarkerArray, 'visualization/tasks', 10)
        self.robot_markers_pub = self.create_publisher(
            MarkerArray, 'visualization/robots', 10)
        self.assignment_markers_pub = self.create_publisher(
            MarkerArray, 'visualization/assignments', 10)
        self.dependency_markers_pub = self.create_publisher(
            MarkerArray, 'visualization/dependencies', 10)
        self.bid_markers_pub = self.create_publisher(
            MarkerArray, 'visualization/bids', 10)
        
        # Initialize data storage
        self.tasks = {}
        self.robot_status = {}
        self.bids = []
        self.auction_status = None
        
        # Visualization settings
        self.task_scale = 0.1
        self.robot_scale = 0.2
        self.arrow_scale = 0.02
        self.text_scale = 0.1
        
        # Colors for different states
        self.colors = {
            'pending': ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8),
            'assigned': ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),
            'in_progress': ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8),
            'completed': ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),
            'failed': ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),
            'robot1': ColorRGBA(r=0.2, g=0.2, b=0.8, a=1.0),
            'robot2': ColorRGBA(r=0.8, g=0.2, b=0.2, a=1.0),
            'dependency': ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5),
            'bid': ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.6)
        }
        
        # Set update timer
        self.timer = self.create_timer(0.1, self.update_visualization)
        
        self.get_logger().info('Auction visualizer initialized')
    
    def task_callback(self, msg):
        """Process task updates"""
        for task in msg.tasks:
            self.tasks[task.id] = task
    
    def robot_status_callback(self, msg):
        """Process robot status updates"""
        self.robot_status[msg.robot_id] = msg
    
    def bid_callback(self, msg):
        """Process bid updates"""
        # Add new bid with timestamp for expiry
        current_time = self.get_clock().now()
        self.bids.append((msg, current_time))
        
        # Remove expired bids (older than 2 seconds)
        self.bids = [(bid, time) for bid, time in self.bids
                    if (current_time - time).nanoseconds < 2e9]
    
    def auction_status_callback(self, msg):
        """Process auction status updates"""
        self.auction_status = msg
    
    def update_visualization(self):
        """Update all visualizations"""
        self.visualize_tasks()
        self.visualize_robots()
        self.visualize_assignments()
        self.visualize_dependencies()
        self.visualize_bids()
    
    def visualize_tasks(self):
        """Visualize tasks as cubes with labels"""
        markers = MarkerArray()
        
        for task_id, task in self.tasks.items():
            # Create cube marker for task
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "tasks"
            marker.id = task_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose = task.position
            
            # Set size
            marker.scale = Vector3(x=self.task_scale, y=self.task_scale, z=self.task_scale)
            
            # Set color based on task status
            if task.status == Task.STATUS_PENDING:
                marker.color = self.colors['pending']
            elif task.status == Task.STATUS_ASSIGNED:
                marker.color = self.colors['assigned']
            elif task.status == Task.STATUS_IN_PROGRESS:
                marker.color = self.colors['in_progress']
            elif task.status == Task.STATUS_COMPLETED:
                marker.color = self.colors['completed']
            elif task.status == Task.STATUS_FAILED:
                marker.color = self.colors['failed']
            
            markers.markers.append(marker)
            
            # Create text marker for task label
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "task_labels"
            text_marker.id = task_id + 1000  # Offset to avoid ID collision
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Set position (slightly above task)
            text_marker.pose = task.position
            text_marker.pose.position.z += self.task_scale
            
            # Set text
            text_marker.text = f"Task {task_id} ({task.execution_time:.1f}s)"
            
            # Set size
            text_marker.scale.z = self.text_scale
            
            # Set color (white)
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            
            markers.markers.append(text_marker)
        
        self.task_markers_pub.publish(markers)
    
    def visualize_robots(self):
        """Visualize robots as cylinders with labels"""
        markers = MarkerArray()
        
        for robot_id, status in self.robot_status.items():
            # Create cylinder marker for robot
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "robots"
            marker.id = robot_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Set position
            marker.pose = status.pose
            
            # Set size
            marker.scale = Vector3(x=self.robot_scale, y=self.robot_scale, z=self.robot_scale/2)
            
            # Set color based on robot ID
            if robot_id == 1:
                marker.color = self.colors['robot1']
            else:
                marker.color = self.colors['robot2']
            
            # If robot has failed, make it red
            if status.failed:
                marker.color = self.colors['failed']
            
            markers.markers.append(marker)
            
            # Create text marker for robot label
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "robot_labels"
            text_marker.id = robot_id + 2000  # Offset to avoid ID collision
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Set position (slightly above robot)
            text_marker.pose = status.pose
            text_marker.pose.position.z += self.robot_scale
            
            # Set text
            text_marker.text = f"Robot {robot_id} (Load: {status.current_workload:.1f})"
            
            # Set size
            text_marker.scale.z = self.text_scale
            
            # Set color (white)
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            
            markers.markers.append(text_marker)
        
        self.robot_markers_pub.publish(markers)
    
    def visualize_assignments(self):
        """Visualize assignments as arrows from robots to tasks"""
        markers = MarkerArray()
        marker_id = 0
        
        for task_id, task in self.tasks.items():
            if task.assigned_robot > 0 and task.status != Task.STATUS_COMPLETED:
                # Find the assigned robot
                if task.assigned_robot in self.robot_status:
                    robot_status = self.robot_status[task.assigned_robot]
                    
                    # Create arrow marker for assignment
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "assignments"
                    marker.id = marker_id
                    marker.type = Marker.ARROW
                    marker.action = Marker.ADD
                    
                    # Set arrow points
                    marker.points = [
                        robot_status.pose.position,
                        task.position.position
                    ]
                    
                    # Set arrow size
                    marker.scale = Vector3(x=self.arrow_scale, y=self.arrow_scale*2, z=self.arrow_scale*2)
                    
                    # Set color based on robot ID
                    if task.assigned_robot == 1:
                        marker.color = self.colors['robot1']
                    else:
                        marker.color = self.colors['robot2']
                    
                    markers.markers.append(marker)
                    marker_id += 1
        
        self.assignment_markers_pub.publish(markers)
    
    def visualize_dependencies(self):
        """Visualize task dependencies as arrows"""
        markers = MarkerArray()
        marker_id = 0
        
        for task_id, task in self.tasks.items():
            for prereq_id in task.prerequisites:
                if prereq_id in self.tasks:
                    prereq_task = self.tasks[prereq_id]
                    
                    # Create arrow marker for dependency
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "dependencies"
                    marker.id = marker_id
                    marker.type = Marker.ARROW
                    marker.action = Marker.ADD
                    
                    # Set arrow points
                    marker.points = [
                        prereq_task.position.position,
                        task.position.position
                    ]
                    
                    # Set arrow size (thinner than assignment arrows)
                    marker.scale = Vector3(x=self.arrow_scale/2, y=self.arrow_scale, z=self.arrow_scale)
                    
                    # Set color
                    marker.color = self.colors['dependency']
                    
                    markers.markers.append(marker)
                    marker_id += 1
        
        self.dependency_markers_pub.publish(markers)
    
    def visualize_bids(self):
        """Visualize recent bids as lines from robots to tasks"""
        markers = MarkerArray()
        marker_id = 0
        
        for bid, _ in self.bids:
            if bid.robot_id in self.robot_status and bid.task_id in self.tasks:
                robot_status = self.robot_status[bid.robot_id]
                task = self.tasks[bid.task_id]
                
                # Create line marker for bid
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "bids"
                marker.id = marker_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                
                # Set line points
                marker.points = [
                    robot_status.pose.position,
                    task.position.position
                ]
                
                # Set line size
                marker.scale.x = self.arrow_scale/4
                
                # Set color
                marker.color = self.colors['bid']
                
                markers.markers.append(marker)
                
                # Create text marker for bid value
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = "bid_values"
                text_marker.id = marker_id + 5000  # Offset to avoid ID collision
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                # Set position (midpoint between robot and task)
                mid_x = (robot_status.pose.position.x + task.position.position.x) / 2
                mid_y = (robot_status.pose.position.y + task.position.position.y) / 2
                mid_z = (robot_status.pose.position.z + task.position.position.z) / 2
                text_marker.pose.position = Point(x=mid_x, y=mid_y, z=mid_z + 0.1)
                
                # Set text
                text_marker.text = f"Bid: {bid.bid_value:.2f}"
                
                # Set size
                text_marker.scale.z = self.text_scale * 0.8
                
                # Set color
                text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8)
                
                markers.markers.append(text_marker)
                marker_id += 1
        
        self.bid_markers_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    
    visualizer = AuctionVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()