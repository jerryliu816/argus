#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Dock, Undock
import logging

logger = logging.getLogger(__name__)

class SimpleRobotController(Node):
    """Simple ROS2 robot controller - no threading"""
    
    def __init__(self):
        super().__init__('web_robot_controller')
        
        # Create publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create action clients for dock/undock
        self.dock_client = ActionClient(self, Dock, '/dock')
        self.undock_client = ActionClient(self, Undock, '/undock')
        
        logger.info("Simple robot controller initialized")
        logger.info(f"Node name: {self.get_name()}")
        logger.info(f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', 'not set')}")
        logger.info(f"RMW_IMPLEMENTATION: {os.environ.get('RMW_IMPLEMENTATION', 'not set')}")
        
        # Test discovery - check if we can see other nodes
        try:
            import time
            time.sleep(2)  # Wait for discovery
            node_names = self.get_node_names()
            logger.info(f"Discovered {len(node_names)} other nodes: {node_names[:5]}...")
            
            # Check for specific nodes we expect
            expected_nodes = ['/motion_control', '/robot_state', '/ui_mgr']
            found_nodes = [name for name in expected_nodes if name in node_names]
            logger.info(f"Found expected robot nodes: {found_nodes}")
            
        except Exception as e:
            logger.error(f"Failed to discover other nodes: {e}")
    
    def publish_twist(self, linear_x: float, linear_y: float, linear_z: float,
                     angular_x: float, angular_y: float, angular_z: float):
        """Publish twist message for robot movement"""
        try:
            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.linear.y = float(linear_y)  
            twist.linear.z = float(linear_z)
            twist.angular.x = float(angular_x)
            twist.angular.y = float(angular_y)
            twist.angular.z = float(angular_z)
            
            self.cmd_vel_publisher.publish(twist)
            logger.info(f"Published twist: linear=({twist.linear.x:.2f}, {twist.linear.y:.2f}, {twist.linear.z:.2f}), angular=({twist.angular.x:.2f}, {twist.angular.y:.2f}, {twist.angular.z:.2f})")
            return True
            
        except Exception as e:
            logger.error(f"Failed to publish twist: {e}")
            return False
    
    def dock(self) -> bool:
        """Send dock command to robot"""
        try:
            if not self.dock_client.wait_for_server(timeout_sec=1.0):
                logger.warning("Dock action server not available")
                return False
            
            goal = Dock.Goal()
            future = self.dock_client.send_goal_async(goal)
            logger.info("Dock command sent")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send dock command: {e}")
            return False
    
    def undock(self) -> bool:
        """Send undock command to robot"""
        try:
            if not self.undock_client.wait_for_server(timeout_sec=1.0):
                logger.warning("Undock action server not available")
                return False
            
            goal = Undock.Goal()
            future = self.undock_client.send_goal_async(goal)
            logger.info("Undock command sent")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send undock command: {e}")
            return False

# Global ROS2 controller instance
_robot_controller = None

def get_robot_controller():
    """Get or create the robot controller"""
    global _robot_controller
    
    if _robot_controller is None:
        try:
            # Ensure ROS2 environment is set
            os.environ['ROS_DOMAIN_ID'] = '0'
            os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
            
            # Initialize ROS2 if not already done
            if not rclpy.ok():
                rclpy.init()
            
            # Create controller
            _robot_controller = SimpleRobotController()
            logger.info("Robot controller created successfully")
            
        except Exception as e:
            logger.error(f"Failed to create robot controller: {e}")
            _robot_controller = None
    
    return _robot_controller

def is_connected():
    """Check if robot controller is available"""
    return _robot_controller is not None and rclpy.ok()

def publish_twist(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
    """Publish twist command"""
    controller = get_robot_controller()
    if controller:
        return controller.publish_twist(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
    return False

def dock():
    """Send dock command"""
    controller = get_robot_controller()
    if controller:
        return controller.dock()
    return False

def undock():
    """Send undock command"""
    controller = get_robot_controller()
    if controller:
        return controller.undock()
    return False

def spin_once():
    """Process ROS2 callbacks"""
    if _robot_controller and rclpy.ok():
        rclpy.spin_once(_robot_controller, timeout_sec=0.001)

def shutdown():
    """Shutdown ROS2"""
    global _robot_controller
    if _robot_controller:
        _robot_controller.destroy_node()
        _robot_controller = None
    if rclpy.ok():
        rclpy.shutdown()