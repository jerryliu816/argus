#!/usr/bin/env python3

import subprocess
import threading
import time
import logging
import os
import signal

logger = logging.getLogger(__name__)

class ROSBridge:
    """Bridge that uses teleop.py subprocess to send ROS commands"""
    
    def __init__(self):
        self.process = None
        self.is_running = False
        self.last_command_time = 0
        self.command_timeout = 0.1  # Stop sending commands after 0.1s
        
    def start(self):
        """Start the teleop.py subprocess"""
        try:
            # Path to teleop.py
            teleop_path = "/home/ubuntu/argus/scripts/teleop.py"
            
            # Create a pseudo-terminal for teleop.py
            import pty
            
            # Create master and slave file descriptors
            master_fd, slave_fd = pty.openpty()
            
            # Start teleop.py with the slave as stdin/stdout
            self.process = subprocess.Popen(
                ['python3', teleop_path],
                stdin=slave_fd,
                stdout=slave_fd,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # Close slave fd in parent process
            os.close(slave_fd)
            
            # Store master fd for writing
            self.master_fd = master_fd
            
            self.is_running = True
            logger.info("ROS bridge started successfully")
            
            # Start background thread to monitor process
            self.monitor_thread = threading.Thread(target=self._monitor_process, daemon=True)
            self.monitor_thread.start()
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to start ROS bridge: {e}")
            return False
    
    def _monitor_process(self):
        """Monitor the subprocess"""
        while self.is_running and self.process:
            if self.process.poll() is not None:
                logger.warning("Teleop process died, restarting...")
                time.sleep(1)
                self.start()
                break
            time.sleep(1)
    
    def send_key(self, key):
        """Send a key command to teleop.py"""
        if not self.is_running or not self.process or not hasattr(self, 'master_fd'):
            logger.warning("ROS bridge not running")
            return False
            
        try:
            # Write key to pseudo-terminal
            os.write(self.master_fd, key.encode())
            self.last_command_time = time.time()
            logger.debug(f"Sent key: {key}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send key {key}: {e}")
            return False
    
    def publish_twist(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        """Convert twist to key commands and send to teleop.py"""
        
        # Map twist values to teleop.py key commands
        key = None
        
        # Simple mapping - extend this as needed
        if linear_x > 0.1:
            if angular_z > 0.1:
                key = 'u'  # forward + left
            elif angular_z < -0.1:
                key = 'o'  # forward + right  
            else:
                key = 'i'  # forward
        elif linear_x < -0.1:
            if angular_z > 0.1:
                key = '.'  # backward + left
            elif angular_z < -0.1:
                key = 'm'  # backward + right
            else:
                key = ','  # backward
        elif angular_z > 0.1:
            key = 'j'  # left turn
        elif angular_z < -0.1:
            key = 'l'  # right turn
        else:
            key = 'k'  # stop
        
        if key:
            success = self.send_key(key)
            logger.info(f"Mapped twist({linear_x:.2f}, {angular_z:.2f}) -> key '{key}', success: {success}")
            return success
        
        return False
    
    def dock(self):
        """Send dock command"""
        return self.send_key('d')
    
    def undock(self):
        """Send undock command"""
        return self.send_key('s')
    
    def stop(self):
        """Stop the bridge"""
        self.is_running = False
        
        if self.process:
            try:
                # Send stop key first
                self.send_key('k')
                time.sleep(0.1)
                
                # Send Ctrl+C to gracefully exit
                self.process.send_signal(signal.SIGINT)
                
                # Wait for graceful shutdown
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    # Force kill if needed
                    self.process.kill()
                    
            except Exception as e:
                logger.error(f"Error stopping process: {e}")
            
            self.process = None
        
        # Close pseudo-terminal
        if hasattr(self, 'master_fd'):
            try:
                os.close(self.master_fd)
                delattr(self, 'master_fd')
            except Exception as e:
                logger.error(f"Error closing master fd: {e}")
        
        logger.info("ROS bridge stopped")
    
    def is_connected(self):
        """Check if bridge is connected"""
        return self.is_running and self.process and self.process.poll() is None

# Global bridge instance
_bridge = None

def get_bridge():
    """Get or create the ROS bridge"""
    global _bridge
    if _bridge is None:
        _bridge = ROSBridge()
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