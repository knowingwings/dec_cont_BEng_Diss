# communication_middleware.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import random
import time
import threading
import numpy as np
from collections import deque

class CommunicationMiddleware(Node):
    """
    Middleware to simulate communication constraints (delay and packet loss)
    between robots in the distributed system.
    """
    
    def __init__(self):
        super().__init__('communication_middleware')
        
        # Parameters for communication constraints
        self.declare_parameter('delay_mean_ms', 0)  # Mean delay in milliseconds
        self.declare_parameter('delay_stddev_ms', 0)  # Standard deviation of delay
        self.declare_parameter('packet_loss_prob', 0.0)  # Probability of packet loss
        
        # Get parameters
        self.delay_mean = self.get_parameter('delay_mean_ms').value / 1000.0  # Convert to seconds
        self.delay_stddev = self.get_parameter('delay_stddev_ms').value / 1000.0
        self.packet_loss_prob = self.get_parameter('packet_loss_prob').value
        
        # Message queue for delayed messages
        self.message_queue = deque()
        
        # Lock for thread safety
        self.queue_lock = threading.Lock()
        
        # Create thread for processing delayed messages
        self.processing_thread = threading.Thread(target=self.process_message_queue)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        
        # Log initialization
        self.get_logger().info(
            f'Communication middleware initialized with delay={self.delay_mean}±{self.delay_stddev}s, '
            f'packet_loss={self.packet_loss_prob}')
    
    def simulate_constraints(self, publisher, msg, topic_type):
        """
        Apply simulated communication constraints to a message.
        
        Parameters:
        -----------
        publisher : rclpy.publisher.Publisher
            The publisher to use for the message
        msg : Any
            The message to publish
        topic_type : type
            The type of the message
        
        Returns:
        --------
        bool
            True if the message was queued (not lost), False otherwise
        """
        # Simulate packet loss
        if random.random() < self.packet_loss_prob:
            return False  # Message lost
        
        # Simulate delay
        if self.delay_mean > 0 or self.delay_stddev > 0:
            # Generate delay from log-normal distribution (more realistic for network delays)
            delay = np.random.lognormal(mean=np.log(self.delay_mean) if self.delay_mean > 0 else -5, 
                                       sigma=self.delay_stddev if self.delay_stddev > 0 else 0.1)
            delay = max(0.001, delay)  # Ensure minimum delay
            
            # Queue message for delayed publishing
            delivery_time = time.time() + delay
            
            with self.queue_lock:
                self.message_queue.append((delivery_time, publisher, msg, topic_type))
                self.message_queue = deque(sorted(self.message_queue, key=lambda x: x[0]))
            
            return True  # Message queued
        else:
            # No delay, publish immediately
            publisher.publish(msg)
            return True  # Message published
    
    def process_message_queue(self):
        """Process delayed messages in the queue when their time comes."""
        while rclpy.ok():
            current_time = time.time()
            
            messages_to_publish = []
            
            # Identify messages that are ready to be published
            with self.queue_lock:
                while self.message_queue and self.message_queue[0][0] <= current_time:
                    messages_to_publish.append(self.message_queue.popleft())
            
            # Publish ready messages
            for _, publisher, msg, _ in messages_to_publish:
                publisher.publish(msg)
            
            # Sleep briefly to prevent high CPU usage
            time.sleep(0.001)