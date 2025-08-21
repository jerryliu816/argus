#!/usr/bin/env python3

import subprocess
import threading
import time
import logging
import os
import json

logger = logging.getLogger(__name__)

class CLIBridge:
    """Bridge that uses ros2 CLI commands directly"""
    
    def __init__(self):
        self.is_running = True
        self.last_publish_time = 0
        
    def start(self):
        """Start the bridge (test ros2 CLI availability)"""
        try:
            # Test if ros2 topic pub works
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=5,
                env=self._get_ros_env()
            )
            
            if result.returncode == 0 and '/cmd_vel' in result.stdout:
                logger.info("CLI bridge initialized successfully - /cmd_vel topic found")
                return True
            else:
                logger.error(f"CLI bridge test failed: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to test CLI bridge: {e}")
            return False
    
    def _get_ros_env(self):
        """Get ROS environment for subprocess"""
        env = os.environ.copy()
        env.update({
            'ROS_DOMAIN_ID': '0',
            'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
            'ROS_DISCOVERY_SERVER': '127.0.0.1:11811',
            'ROS_LOCALHOST_ONLY': '0'
        })
        return env
    
    def publish_twist(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        """Publish twist using ros2 topic pub command"""
        if not self.is_running:
            return False
            
        try:
            # Create twist message as YAML
            twist_yaml = f"""linear:
  x: {linear_x}
  y: {linear_y}
  z: {linear_z}
angular:
  x: {angular_x}
  y: {angular_y}
  z: {angular_z}"""
            
            # Use ros2 topic pub --once to publish single message
            cmd = [
                'ros2', 'topic', 'pub', '--once',
                '/cmd_vel', 'geometry_msgs/msg/Twist',
                twist_yaml
            ]
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=3,
                env=self._get_ros_env()
            )
            
            self.last_publish_time = time.time()
            
            if result.returncode == 0:
                logger.debug(f"Published twist via CLI: linear=({linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}), angular=({angular_x:.2f}, {angular_y:.2f}, {angular_z:.2f})")
                return True
            else:
                logger.error(f"CLI publish failed: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            logger.warning("CLI publish timed out")
            return False
        except Exception as e:
            logger.error(f"Failed to publish via CLI: {e}")
            return False
    
    def dock(self):
        """Send dock command via CLI"""
        try:
            cmd = [
                'ros2', 'action', 'send_goal', '/dock',
                'irobot_create_msgs/action/Dock', '{}'
            ]
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=5,
                env=self._get_ros_env()
            )
            
            if result.returncode == 0:
                logger.info("Dock command sent via CLI")
                return True
            else:
                logger.warning(f"CLI dock failed: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to send dock via CLI: {e}")
            return False
    
    def undock(self):
        """Send undock command via CLI"""
        try:
            cmd = [
                'ros2', 'action', 'send_goal', '/undock',
                'irobot_create_msgs/action/Undock', '{}'
            ]
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=5,
                env=self._get_ros_env()
            )
            
            if result.returncode == 0:
                logger.info("Undock command sent via CLI")
                return True
            else:
                logger.warning(f"CLI undock failed: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to send undock via CLI: {e}")
            return False
    
    def stop(self):
        """Stop the bridge"""
        self.is_running = False
        logger.info("CLI bridge stopped")
    
    def is_connected(self):
        """Check if bridge is connected"""
        return self.is_running

# Global bridge instance
_bridge = None

def get_bridge():
    """Get or create the CLI bridge"""
    global _bridge
    if _bridge is None:
        _bridge = CLIBridge()
        _bridge.start()
    return _bridge

def is_connected():
    """Check if bridge is connected"""
    bridge = get_bridge()
    return bridge.is_connected()

def publish_twist(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
    """Publish twist via bridge"""
    bridge = get_bridge()
    return bridge.publish_twist(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)

def dock():
    """Send dock command via bridge"""
    bridge = get_bridge()
    return bridge.dock()

def undock():
    """Send undock command via bridge"""
    bridge = get_bridge()
    return bridge.undock()

def shutdown():
    """Shutdown bridge"""
    global _bridge
    if _bridge:
        _bridge.stop()
        _bridge = None