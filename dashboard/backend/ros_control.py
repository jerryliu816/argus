#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Dock, Undock
import threading
import time
import logging
import queue

logger = logging.getLogger(__name__)

class WebRobotNode(Node):
    """ROS2 Node for web robot control"""
    
    def __init__(self):
        super().__init__('web_robot_controller')
        
        # Create publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create action clients for dock/undock
        self.dock_client = ActionClient(self, Dock, '/dock')
        self.undock_client = ActionClient(self, Undock, '/undock')

class RobotController:
    """ROS2 interface for robot control - minimal and focused"""
    
    def __init__(self):
        # Initialize ROS2 in a separate thread to avoid blocking
        self._ros_thread = None
        self._should_stop = False
        self._connected = False
        self._node = None
        
        # Command queue for thread-safe communication
        self._command_queue = queue.Queue()
        
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
            self._node = WebRobotNode()
            
            self._connected = True
            logger.info("ROS2 robot controller initialized successfully")
            
            # Spin and process commands
            while not self._should_stop and rclpy.ok():
                # Process any queued commands
                try:
                    commands_processed = 0
                    while not self._command_queue.empty():
                        command = self._command_queue.get_nowait()
                        logger.info(f"ROS worker processing command: {command['type']}")
                        self._process_command(command)
                        commands_processed += 1
                    
                    if commands_processed > 0:
                        logger.info(f"Processed {commands_processed} commands")
                        
                except queue.Empty:
                    pass
                
                # Spin ROS2
                rclpy.spin_once(self._node, timeout_sec=0.1)
                
        except Exception as e:
            logger.error(f"ROS2 initialization failed: {e}")
            self._connected = False
        finally:
            if self._node and rclpy.ok():
                self._node.destroy_node()
                rclpy.shutdown()
    
    def _process_command(self, command):
        """Process a command in the ROS2 thread"""
        cmd_type = command['type']
        
        if cmd_type == 'twist':
            twist = Twist()
            twist.linear.x = float(command['linear_x'])
            twist.linear.y = float(command['linear_y'])  
            twist.linear.z = float(command['linear_z'])
            twist.angular.x = float(command['angular_x'])
            twist.angular.y = float(command['angular_y'])
            twist.angular.z = float(command['angular_z'])
            
            self._node.cmd_vel_publisher.publish(twist)
            logger.debug(f"Published twist: {twist.linear.x:.2f}, {twist.angular.z:.2f}")
            
        elif cmd_type == 'dock':
            if self._node.dock_client.wait_for_server(timeout_sec=1.0):
                goal = Dock.Goal()
                self._node.dock_client.send_goal_async(goal)
                logger.info("Dock command sent")
            else:
                logger.warning("Dock action server not available")
                
        elif cmd_type == 'undock':
            if self._node.undock_client.wait_for_server(timeout_sec=1.0):
                goal = Undock.Goal()
                self._node.undock_client.send_goal_async(goal)
                logger.info("Undock command sent")
            else:
                logger.warning("Undock action server not available")
    
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
            command = {
                'type': 'twist',
                'linear_x': linear_x,
                'linear_y': linear_y,
                'linear_z': linear_z,
                'angular_x': angular_x,
                'angular_y': angular_y,
                'angular_z': angular_z
            }
            self._command_queue.put(command)
            logger.info(f"Queued twist command: queue size = {self._command_queue.qsize()}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to queue twist command: {e}")
            return False
    
    def dock(self) -> bool:
        """Send dock command to robot"""
        if not self.is_connected():
            logger.warning("Cannot dock - ROS2 not connected")
            return False
            
        try:
            command = {'type': 'dock'}
            self._command_queue.put(command)
            return True
            
        except Exception as e:
            logger.error(f"Failed to queue dock command: {e}")
            return False
    
    def undock(self) -> bool:
        """Send undock command to robot"""
        if not self.is_connected():
            logger.warning("Cannot undock - ROS2 not connected")
            return False
            
        try:
            command = {'type': 'undock'}
            self._command_queue.put(command)
            return True
            
        except Exception as e:
            logger.error(f"Failed to queue undock command: {e}")
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