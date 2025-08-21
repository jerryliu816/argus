#!/usr/bin/env python3

# Exact copy of teleop.py structure but minimal

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_web_publisher')  # Different name to avoid conflicts
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Minimal publisher created')

    def test_publish(self):
        """Test publishing like teleop.py does"""
        # Publish forward
        twist = Twist()
        twist.linear.x = 0.1
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        self.pub.publish(twist)
        self.get_logger().info('Published forward command')
        
        # Wait a bit
        time.sleep(1)
        
        # Publish stop
        twist.linear.x = 0.0
        self.pub.publish(twist)
        self.get_logger().info('Published stop command')

def main():
    print("Starting minimal publisher test...")
    
    # Initialize exactly like teleop.py
    rclpy.init()
    
    publisher = MinimalPublisher()
    
    try:
        # Test publishing
        publisher.test_publish()
        
        # Keep node alive briefly
        for i in range(10):
            rclpy.spin_once(publisher, timeout_sec=0.1)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()
        print("Test completed")

if __name__ == '__main__':
    main()