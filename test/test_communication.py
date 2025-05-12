#!/usr/bin/env python3

import unittest
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread

from decentralized_control.communication_middleware import CommunicationMiddleware

class TestPublisher(Node):
    """Test publisher node for communication tests."""
    
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(String, 'test_topic', 10)
    
    def send_message(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)

class TestSubscriber(Node):
    """Test subscriber node for communication tests."""
    
    def __init__(self):
        super().__init__('test_subscriber')
        self.subscription = self.create_subscription(
            String, 'test_topic', self.message_callback, 10)
        self.received_messages = []
        self.message_times = []
    
    def message_callback(self, msg):
        self.received_messages.append(msg.data)
        self.message_times.append(time.time())

class TestCommunication(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.middleware = CommunicationMiddleware()
        cls.publisher = TestPublisher()
        cls.subscriber = TestSubscriber()
        
        # Start spinning in a separate thread
        cls.spin_thread = Thread(target=lambda: rclpy.spin(cls.middleware))
        cls.spin_thread.daemon = True
        cls.spin_thread.start()
        
        # Wait for nodes to initialize
        time.sleep(1)
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.subscriber.received_messages = []
        self.subscriber.message_times = []
    
    def test_delay(self):
        """Test that communication delay is applied correctly."""
        # Configure middleware for 200ms delay
        self.middleware.delay_mean = 0.2
        self.middleware.delay_stddev = 0.02
        self.middleware.packet_loss_prob = 0.0
        
        # Send test message
        start_time = time.time()
        msg = String()
        msg.data = "test_delay"
        self.middleware.simulate_constraints(
            self.publisher.publisher, msg, String)
        
        # Wait for message to be delivered with delay (plus buffer)
        time.sleep(0.5)
        
        # Verify message was received
        self.assertEqual(len(self.subscriber.received_messages), 1)
        self.assertEqual(self.subscriber.received_messages[0], "test_delay")
        
        # Verify delay
        delay = self.subscriber.message_times[0] - start_time
        self.assertGreater(delay, 0.1)  # Should be delayed at least 100ms
    
    def test_packet_loss(self):
        """Test that packet loss is simulated correctly."""
        # Configure middleware for 100% packet loss
        self.middleware.delay_mean = 0.0
        self.middleware.delay_stddev = 0.0
        self.middleware.packet_loss_prob = 1.0
        
        # Send test message
        msg = String()
        msg.data = "test_packet_loss"
        result = self.middleware.simulate_constraints(
            self.publisher.publisher, msg, String)
        
        # Verify message was considered "lost"
        self.assertFalse(result)
        
        # Wait a moment and verify no message was received
        time.sleep(0.1)
        self.assertEqual(len(self.subscriber.received_messages), 0)
    
    def test_realistic_settings(self):
        """Test with realistic communication settings."""
        # Configure middleware for realistic settings
        self.middleware.delay_mean = 0.05
        self.middleware.delay_stddev = 0.02
        self.middleware.packet_loss_prob = 0.2
        
        # Send multiple messages
        messages_sent = 0
        for i in range(100):
            msg = String()
            msg.data = f"test_message_{i}"
            result = self.middleware.simulate_constraints(
                self.publisher.publisher, msg, String)
            if result:
                messages_sent += 1
            time.sleep(0.01)
        
        # Wait for messages to be delivered
        time.sleep(0.5)
        
        # Verify some messages were lost but not all
        self.assertLess(len(self.subscriber.received_messages), 100)
        self.assertEqual(len(self.subscriber.received_messages), messages_sent)
        
        # Verify message order is preserved
        for i in range(len(self.subscriber.received_messages) - 1):
            current_id = int(self.subscriber.received_messages[i].split('_')[-1])
            next_id = int(self.subscriber.received_messages[i+1].split('_')[-1])
            self.assertLess(current_id, next_id)

if __name__ == '__main__':
    unittest.main()