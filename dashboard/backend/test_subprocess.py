#!/usr/bin/env python3

import subprocess
import os
import time

def test_ros_with_subprocess():
    """Test ROS2 by running it exactly like teleop.py would"""
    
    # Create a simple ROS2 publisher script
    test_script = '''
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestPublisher(Node):
    def __init__(self):
        super().__init__('web_test_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Test publisher created')
        
        # Wait for discovery
        import time
        time.sleep(2)
        
        # Check what nodes we can see
        node_names = self.get_node_names()
        self.get_logger().info(f'Can see {len(node_names)} nodes: {node_names[:5]}')
        
        # Publish a test message
        twist = Twist()
        twist.linear.x = 0.1
        self.publisher.publish(twist)
        self.get_logger().info('Published test message')
        
        # Stop message
        time.sleep(1)
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Published stop message')

def main():
    rclpy.init()
    node = TestPublisher()
    
    # Spin for a bit
    for i in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
    
    # Write the test script
    with open('/tmp/ros_test.py', 'w') as f:
        f.write(test_script)
    
    # Run it with the same environment as a bash script would
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '0'
    env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    
    # Try different combinations
    configurations = [
        {},  # No additional config
        {'ROS_DISCOVERY_SERVER': '127.0.0.1:11811'},
        {'ROS_LOCALHOST_ONLY': '0'},
        {'ROS_DISCOVERY_SERVER': '127.0.0.1:11811', 'ROS_LOCALHOST_ONLY': '0'},
    ]
    
    for i, config in enumerate(configurations):
        print(f"\n=== Test {i+1}: {config} ===")
        
        test_env = env.copy()
        test_env.update(config)
        
        try:
            # Source ROS and run the test
            cmd = [
                'bash', '-c', 
                'source /opt/ros/humble/setup.bash && python3 /tmp/ros_test.py'
            ]
            
            result = subprocess.run(
                cmd, 
                env=test_env, 
                capture_output=True, 
                text=True, 
                timeout=10
            )
            
            print("STDOUT:")
            print(result.stdout)
            if result.stderr:
                print("STDERR:")
                print(result.stderr)
            print(f"Return code: {result.returncode}")
            
        except subprocess.TimeoutExpired:
            print("Test timed out")
        except Exception as e:
            print(f"Test failed: {e}")

if __name__ == '__main__':
    test_ros_with_subprocess()