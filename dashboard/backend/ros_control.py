#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.action import Dock, Undock
import threading
import time
import logging
import queue
import math

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
        
        # Battery monitoring
        self.battery_percentage = None
        self.battery_subscriber = self.create_subscription(
            BatteryState,
            '/battery_state',
            self._battery_callback,
            10
        )
        
    def _battery_callback(self, msg):
        """Callback for battery state messages"""
        if not math.isnan(msg.percentage):  # Check for NaN
            # Convert from 0-1 range to 0-100 percentage
            self.battery_percentage = int(msg.percentage * 100)
            logger.debug(f"Battery percentage updated: {self.battery_percentage}%")
    
    def get_battery_percentage(self):
        """Get current battery percentage"""
        return self.battery_percentage

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
            # Ensure ROS2 environment is set
            os.environ['ROS_DOMAIN_ID'] = '0'
            os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
            
            logger.info(f"Setting ROS environment: DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}, RMW={os.environ.get('RMW_IMPLEMENTATION')}")
            
            # Initialize ROS2
            rclpy.init()
            
            # Create node
            self._node = WebRobotNode()
            
            self._connected = True
            logger.info("ROS2 robot controller initialized successfully")
            logger.info(f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', 'not set')}")
            logger.info(f"RMW_IMPLEMENTATION: {os.environ.get('RMW_IMPLEMENTATION', 'not set')}")
            logger.info(f"Node name: {self._node.get_name()}")
            logger.info(f"Publisher topic: /cmd_vel")
            
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
            try:
                twist = Twist()
                twist.linear.x = float(command['linear_x'])
                twist.linear.y = float(command['linear_y'])  
                twist.linear.z = float(command['linear_z'])
                twist.angular.x = float(command['angular_x'])
                twist.angular.y = float(command['angular_y'])
                twist.angular.z = float(command['angular_z'])
                
                logger.info(f"About to publish twist: linear=({twist.linear.x:.2f}, {twist.linear.y:.2f}, {twist.linear.z:.2f}), angular=({twist.angular.x:.2f}, {twist.angular.y:.2f}, {twist.angular.z:.2f})")
                
                self._node.cmd_vel_publisher.publish(twist)
                logger.info(f"Successfully published twist message")
                
            except Exception as e:
                logger.error(f"Failed to publish twist in _process_command: {e}")
            
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
    
    def get_battery_percentage(self) -> int:
        """Get current battery percentage"""
        if not self.is_connected() or not self._node:
            return None
        
        return self._node.get_battery_percentage()
    
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