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
        self.battery_percentage = None
        self.battery_thread = None
        
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
                # Start battery monitoring thread
                self._start_battery_monitoring()
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
    
    def _start_battery_monitoring(self):
        """Start background thread to monitor battery status"""
        if self.battery_thread is None or not self.battery_thread.is_alive():
            self.battery_thread = threading.Thread(target=self._monitor_battery, daemon=True)
            self.battery_thread.start()
            logger.info("Battery monitoring thread started")
    
    def _monitor_battery(self):
        """Monitor battery status in background thread"""
        while self.is_running:
            try:
                # Use ros2 topic echo --once to get latest battery status
                result = subprocess.run(
                    ['ros2', 'topic', 'echo', '/battery_state', '--once'],
                    capture_output=True,
                    text=True,
                    timeout=10,
                    env=self._get_ros_env()
                )
                
                if result.returncode == 0 and result.stdout:
                    # Parse the battery percentage from the output
                    lines = result.stdout.split('\n')
                    for line in lines:
                        if line.strip().startswith('percentage:'):
                            try:
                                percentage_str = line.split(':')[1].strip()
                                percentage = float(percentage_str)
                                # Convert from 0-1 range to 0-100 percentage
                                self.battery_percentage = int(percentage * 100)
                                logger.debug(f"Battery percentage updated: {self.battery_percentage}%")
                                break
                            except (ValueError, IndexError) as e:
                                logger.warning(f"Failed to parse battery percentage: {e}")
                else:
                    logger.warning("Failed to get battery status")
                    
            except subprocess.TimeoutExpired:
                logger.warning("Battery status check timed out")
            except Exception as e:
                logger.error(f"Error monitoring battery: {e}")
            
            # Wait 5 seconds before next check
            time.sleep(5)
    
    def get_battery_percentage(self):
        """Get current battery percentage"""
        return self.battery_percentage
    
    def _monitor_battery_once(self):
        """Check battery status once (for debugging)"""
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'echo', '/battery_state', '--once'],
                capture_output=True,
                text=True,
                timeout=10,
                env=self._get_ros_env()
            )
            
            if result.returncode == 0 and result.stdout:
                lines = result.stdout.split('\n')
                for line in lines:
                    if line.strip().startswith('percentage:'):
                        try:
                            percentage_str = line.split(':')[1].strip()
                            percentage = float(percentage_str)
                            self.battery_percentage = int(percentage * 100)
                            logger.info(f"Manual battery check: {self.battery_percentage}%")
                            return self.battery_percentage
                        except (ValueError, IndexError) as e:
                            logger.warning(f"Failed to parse battery percentage: {e}")
            else:
                logger.warning("Manual battery check failed")
                
        except Exception as e:
            logger.error(f"Manual battery check error: {e}")
        
        return None
    
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
        if self.battery_thread and self.battery_thread.is_alive():
            self.battery_thread.join(timeout=2)
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

def get_battery_percentage():
    """Get current battery percentage via bridge"""
    bridge = get_bridge()
    return bridge.get_battery_percentage()

def shutdown():
    """Shutdown bridge"""
    global _bridge
    if _bridge:
        _bridge.stop()
        _bridge = None