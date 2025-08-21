#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Dock, Undock
import threading
import time
import logging

logger = logging.getLogger(__name__)

class RobotController(Node):
    """ROS2 interface for robot control - minimal and focused"""
    
    def __init__(self):
        # Initialize ROS2 in a separate thread to avoid blocking
        self._ros_thread = None
        self._should_stop = False
        self._connected = False
        
        # Start ROS2 in background thread
        self._start_ros_thread()
        
        # Wait for initialization
        time.sleep(1.0)
    
    def _start_ros_thread(self):
        """Start ROS2 in a separate thread"""
        self._ros_thread = threading.Thread(target=self._ros_worker, daemon=True)
        self._ros_thread.start()
    
    def _ros_worker(self):
        """ROS2 worker thread"""
        try:
            # Initialize ROS2
            rclpy.init()
            
            # Create node
            super().__init__('web_robot_controller')
            
            # Create publisher for cmd_vel
            self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
            
            # Create action clients for dock/undock
            self.dock_client = ActionClient(self, Dock, '/dock')
            self.undock_client = ActionClient(self, Undock, '/undock')
            
            self._connected = True
            logger.info("ROS2 robot controller initialized successfully")
            
            # Spin in this thread
            while not self._should_stop and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                
        except Exception as e:
            logger.error(f"ROS2 initialization failed: {e}")
            self._connected = False
        finally:
            if rclpy.ok():
                self.destroy_node()
                rclpy.shutdown()
    
    def is_connected(self) -> bool:
        """Check if ROS2 connection is active"""
        return self._connected and not self._should_stop
    
    def publish_twist(self, linear_x: float, linear_y: float, linear_z: float,
                     angular_x: float, angular_y: float, angular_z: float):
        """Publish twist message for robot movement"""
        if not self.is_connected():
            logger.warning("Cannot publish twist - ROS2 not connected")
            return False
            
        try:
            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.linear.y = float(linear_y)  
            twist.linear.z = float(linear_z)
            twist.angular.x = float(angular_x)
            twist.angular.y = float(angular_y)
            twist.angular.z = float(angular_z)
            
            self.cmd_vel_publisher.publish(twist)
            return True
            
        except Exception as e:
            logger.error(f"Failed to publish twist: {e}")
            return False
    
    def dock(self) -> bool:
        """Send dock command to robot"""
        if not self.is_connected():
            logger.warning("Cannot dock - ROS2 not connected")
            return False
            
        try:
            # Check if dock action server is available
            if not self.dock_client.wait_for_server(timeout_sec=1.0):
                logger.warning("Dock action server not available")
                return False
            
            # Send dock goal
            goal = Dock.Goal()
            future = self.dock_client.send_goal_async(goal)
            logger.info("Dock command sent")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send dock command: {e}")
            return False
    
    def undock(self) -> bool:
        """Send undock command to robot"""
        if not self.is_connected():
            logger.warning("Cannot undock - ROS2 not connected")
            return False
            
        try:
            # Check if undock action server is available
            if not self.undock_client.wait_for_server(timeout_sec=1.0):
                logger.warning("Undock action server not available")
                return False
            
            # Send undock goal
            goal = Undock.Goal()
            future = self.undock_client.send_goal_async(goal)
            logger.info("Undock command sent")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send undock command: {e}")
            return False
    
    def stop(self):
        """Stop the robot controller"""
        logger.info("Stopping robot controller...")
        
        # Send stop command
        if self.is_connected():
            self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        
        # Signal thread to stop
        self._should_stop = True
        
        # Wait for thread to finish
        if self._ros_thread and self._ros_thread.is_alive():
            self._ros_thread.join(timeout=2.0)
        
        self._connected = False
        logger.info("Robot controller stopped")