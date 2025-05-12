#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import time
from threading import Lock
import queue
from std_msgs.msg import Float64MultiArray
from dec_control.msg import RobotState, NetworkTopology

class CommunicationMiddleware(Node):
    """Simulates network conditions and handles message delivery."""
    
    def __init__(self):
        super().__init__('communication_middleware')
        
        # Initialize network simulation parameters
        self.declare_parameter('delay_mean_ms', 10)
        self.declare_parameter('delay_stddev_ms', 2)
        self.declare_parameter('packet_loss_prob', 0.0)
        self.declare_parameter('delay_pattern', 'constant')
        self.declare_parameter('base_delay_ms', 0)
        self.declare_parameter('amplitude_ms', 0)
        self.declare_parameter('period_s', 1.0)
        self.declare_parameter('burst_interval_s', 0.0)
        self.declare_parameter('burst_duration_s', 0.0)
        self.declare_parameter('burst_multiplier', 1.0)
        
        # Message queues for delayed delivery
        self.message_queues = {}
        self.queue_lock = Lock()
        
        # Network metrics publishers
        self.latency_pub = self.create_publisher(
            Float64MultiArray, 'network_latency', 10)
        self.message_rate_pub = self.create_publisher(
            Float64MultiArray, 'message_rate', 10)
        self.packet_loss_pub = self.create_publisher(
            Float64MultiArray, 'packet_loss', 10)
        
        # Message delivery timer
        self.timer = self.create_timer(0.01, self.process_message_queues)
        
        # Message rate tracking
        self.message_counts = {}
        self.last_rate_update = time.time()
        self.rate_window = 1.0  # 1 second window for rate calculation
        
        self.get_logger().info('Communication middleware initialized')
    
    def configure(self, **params):
        """Configure network simulation parameters."""
        for param_name, value in params.items():
            self.set_parameter(rclpy.Parameter(param_name, value=value))
        
        self.get_logger().info('Network parameters updated')
    
    def calculate_current_delay(self):
        """Calculate current network delay based on pattern."""
        base_delay = self.get_parameter('base_delay_ms').value
        pattern = self.get_parameter('delay_pattern').value
        
        if pattern == 'constant':
            return self.get_parameter('delay_mean_ms').value / 1000.0
        
        current_time = time.time()
        period = self.get_parameter('period_s').value
        amplitude = self.get_parameter('amplitude_ms').value / 1000.0
        
        if pattern == 'sinusoidal':
            phase = 2 * np.pi * (current_time % period) / period
            return base_delay/1000.0 + amplitude * np.sin(phase)
        
        elif pattern == 'square_wave':
            phase = (current_time % period) / period
            return base_delay/1000.0 + amplitude * (1 if phase < 0.5 else -1)
        
        elif pattern == 'sawtooth':
            phase = (current_time % period) / period
            return base_delay/1000.0 + amplitude * (2 * phase - 1)
        
        elif pattern == 'burst':
            interval = self.get_parameter('burst_interval_s').value
            duration = self.get_parameter('burst_duration_s').value
            multiplier = self.get_parameter('burst_multiplier').value
            
            cycle_time = current_time % interval
            if cycle_time < duration:
                return base_delay/1000.0 * multiplier
            return base_delay/1000.0
        
        return base_delay/1000.0
    
    def simulate_message_delivery(self, topic_name, msg, publisher):
        """Simulate network effects on message delivery."""
        # Check for packet loss
        if np.random.random() < self.get_parameter('packet_loss_prob').value:
            # Record packet loss
            self.publish_packet_loss(msg.id if hasattr(msg, 'id') else 0, 1.0)
            return
        
        # Calculate delay
        base_delay = self.calculate_current_delay()
        stddev = self.get_parameter('delay_stddev_ms').value / 1000.0
        delay = np.random.normal(base_delay, stddev)
        delay = max(0.0, delay)  # Ensure non-negative delay
        
        # Record latency
        self.publish_latency(msg.id if hasattr(msg, 'id') else 0, delay)
        
        # Queue message for delayed delivery
        delivery_time = time.time() + delay
        with self.queue_lock:
            if topic_name not in self.message_queues:
                self.message_queues[topic_name] = queue.PriorityQueue()
            self.message_queues[topic_name].put((delivery_time, msg, publisher))
        
        # Update message rate tracking
        robot_id = msg.id if hasattr(msg, 'id') else 0
        if robot_id not in self.message_counts:
            self.message_counts[robot_id] = 0
        self.message_counts[robot_id] += 1
    
    def process_message_queues(self):
        """Process queued messages for delivery."""
        current_time = time.time()
        
        # Update message rates periodically
        if current_time - self.last_rate_update >= self.rate_window:
            self.update_message_rates()
            self.last_rate_update = current_time
        
        with self.queue_lock:
            for topic_name in list(self.message_queues.keys()):
                queue = self.message_queues[topic_name]
                
                while not queue.empty():
                    delivery_time, msg, publisher = queue.queue[0]
                    
                    if current_time >= delivery_time:
                        queue.get()  # Remove from queue
                        publisher.publish(msg)  # Deliver message
                    else:
                        break  # Messages are ordered by delivery time
    
    def create_delayed_publisher(self, msg_type, topic, qos_profile=None):
        """Create a publisher that simulates network effects."""
        if qos_profile is None:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        
        # Create actual publisher
        real_publisher = self.create_publisher(msg_type, topic, qos_profile)
        
        # Return wrapper function that simulates network effects
        def delayed_publish(msg):
            self.simulate_message_delivery(topic, msg, real_publisher)
        
        return delayed_publish
    
    def create_delayed_subscription(self, msg_type, topic, callback, qos_profile=None):
        """Create a subscription that simulates network effects."""
        if qos_profile is None:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        
        # Create wrapper callback that simulates network effects
        def delayed_callback(msg):
            self.simulate_message_delivery(topic, msg, callback)
        
        return self.create_subscription(msg_type, topic, delayed_callback, qos_profile)
    
    def publish_latency(self, robot_id, latency):
        """Publish latency measurement."""
        msg = Float64MultiArray()
        msg.data = [float(robot_id), latency]
        self.latency_pub.publish(msg)
    
    def publish_packet_loss(self, robot_id, loss_rate):
        """Publish packet loss measurement."""
        msg = Float64MultiArray()
        msg.data = [float(robot_id), loss_rate]
        self.packet_loss_pub.publish(msg)
    
    def update_message_rates(self):
        """Calculate and publish message rates."""
        current_time = time.time()
        window = current_time - self.last_rate_update
        
        for robot_id, count in self.message_counts.items():
            rate = count / window if window > 0 else 0
            
            msg = Float64MultiArray()
            msg.data = [float(robot_id), rate]
            self.message_rate_pub.publish(msg)
        
        # Reset counts
        self.message_counts.clear()

def main(args=None):
    rclpy.init(args=args)
    middleware = CommunicationMiddleware()
    rclpy.spin(middleware)
    middleware.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()