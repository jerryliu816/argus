#!/usr/bin/env python3

import os
import sys
import time

# Set ROS2 environment
os.environ['ROS_DOMAIN_ID'] = '0'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

print(f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID')}")
print(f"RMW_IMPLEMENTATION: {os.environ.get('RMW_IMPLEMENTATION')}")

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    print("✅ ROS2 imports successful")
except ImportError as e:
    print(f"❌ ROS2 import failed: {e}")
    sys.exit(1)

class TestNode(Node):
    def __init__(self):
        super().__init__('test_web_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        print(f"✅ Node created: {self.get_name()}")
        
        # Wait for discovery
        time.sleep(3)
        
        # Check discovery
        try:
            node_names = self.get_node_names()
            print(f"🔍 Discovered {len(node_names)} nodes")
            print(f"   First 5: {node_names[:5]}")
            
            # Check for robot nodes
            robot_nodes = [name for name in node_names if 'motion_control' in name or 'robot_state' in name]
            print(f"🤖 Found robot nodes: {robot_nodes}")
            
        except Exception as e:
            print(f"❌ Discovery failed: {e}")

def main():
    print("🚀 Testing ROS2 connection...")
    
    try:
        rclpy.init()
        node = TestNode()
        
        print("📡 Publishing test message...")
        twist = Twist()
        twist.linear.x = 0.1
        node.cmd_vel_publisher.publish(twist)
        print("✅ Message published")
        
        # Spin briefly to process
        for i in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
        
        print("🛑 Stopping...")
        twist.linear.x = 0.0
        node.cmd_vel_publisher.publish(twist)
        
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Test completed")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()