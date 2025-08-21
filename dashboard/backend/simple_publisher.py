#!/usr/bin/env python3

import sys
import os

# Set environment exactly like the working system
os.environ['ROS_DOMAIN_ID'] = '0'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ['ROS_DISCOVERY_SERVER'] = '127.0.0.1:11811'
os.environ['ROS_LOCALHOST_ONLY'] = '0'

def publish_twist(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
    """Publish a single twist message and exit"""
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        
        class QuickPublisher(Node):
            def __init__(self):
                super().__init__('quick_publisher')
                self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
                
            def publish_once(self, lx, ly, lz, ax, ay, az):
                twist = Twist()
                twist.linear.x = float(lx)
                twist.linear.y = float(ly)
                twist.linear.z = float(lz)
                twist.angular.x = float(ax)
                twist.angular.y = float(ay)
                twist.angular.z = float(az)
                
                self.publisher.publish(twist)
                return True
        
        # Quick publish and exit
        rclpy.init()
        node = QuickPublisher()
        
        success = node.publish_once(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
        
        # Brief spin to ensure message is sent
        for i in range(5):
            rclpy.spin_once(node, timeout_sec=0.01)
        
        node.destroy_node()
        rclpy.shutdown()
        
        return success
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return False

if __name__ == '__main__':
    if len(sys.argv) != 7:
        print("Usage: simple_publisher.py linear_x linear_y linear_z angular_x angular_y angular_z")
        sys.exit(1)
    
    args = [float(x) for x in sys.argv[1:7]]
    success = publish_twist(*args)
    sys.exit(0 if success else 1)