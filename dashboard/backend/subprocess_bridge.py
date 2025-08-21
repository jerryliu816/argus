#!/usr/bin/env python3

import subprocess
import threading
import time
import logging
import os

logger = logging.getLogger(__name__)

class SubprocessBridge:
    """Bridge that uses subprocess calls to simple_publisher.py"""
    
    def __init__(self):
        self.publisher_path = "/home/ubuntu/argus/dashboard/backend/simple_publisher.py"
        self.is_running = True
        self.last_publish_time = 0
        
    def start(self):
        """Start the bridge (no persistent process needed)"""
        # Test if the publisher script works
        try:
            result = subprocess.run(
                ['python3', self.publisher_path, '0', '0', '0', '0', '0', '0'],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0:
                logger.info("Subprocess bridge initialized successfully")
                return True
            else:
                logger.error(f"Publisher test failed: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to test publisher: {e}")
            return False
    
    def publish_twist(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        """Publish twist by calling subprocess"""
        if not self.is_running:
            return False
            
        try:
            # Convert to strings for subprocess
            args = [
                'python3', self.publisher_path,
                str(linear_x), str(linear_y), str(linear_z),
                str(angular_x), str(angular_y), str(angular_z)
            ]
            
            # Run the publisher subprocess
            result = subprocess.run(
                args,
                capture_output=True,
                text=True,
                timeout=2  # Quick timeout
            )
            
            self.last_publish_time = time.time()
            
            if result.returncode == 0:
                logger.debug(f"Published twist: linear=({linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}), angular=({angular_x:.2f}, {angular_y:.2f}, {angular_z:.2f})")
                return True
            else:
                logger.error(f"Publisher failed: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            logger.warning("Publisher subprocess timed out")
            return False
        except Exception as e:
            logger.error(f"Failed to publish twist: {e}")
            return False
    
    def dock(self):
        """Send dock command (not implemented for subprocess approach)"""
        logger.warning("Dock command not implemented in subprocess bridge")
        return False
    
    def undock(self):
        """Send undock command (not implemented for subprocess approach)"""
        logger.warning("Undock command not implemented in subprocess bridge")
        return False
    
    def stop(self):
        """Stop the bridge"""
        self.is_running = False
        logger.info("Subprocess bridge stopped")
    
    def is_connected(self):
        """Check if bridge is connected"""
        return self.is_running

# Global bridge instance
_bridge = None

def get_bridge():
    """Get or create the subprocess bridge"""
    global _bridge
    if _bridge is None:
        _bridge = SubprocessBridge()
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