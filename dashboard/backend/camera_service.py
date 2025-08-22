#!/usr/bin/env python3

import cv2
import depthai as dai
import threading
import time
import logging
import io
from typing import Optional

logger = logging.getLogger(__name__)

class CameraService:
    """Direct DepthAI camera service for web dashboard"""
    
    def __init__(self):
        self.device = None
        self.pipeline = None
        self.q_rgb = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.device_lock = threading.Lock()
        self._should_stop = False
        self._connected = False
        self._camera_thread = None
        self._reconnect_attempts = 0
        self._max_reconnect_attempts = 5
        
        # Initialize camera
        self._initialize_camera()
        
        # Start capture thread
        self._start_camera_thread()
    
    def _initialize_camera(self):
        """Initialize DepthAI camera pipeline"""
        try:
            # Create pipeline (same as rgb_preview.py)
            self.pipeline = dai.Pipeline()
            
            # Define source and output
            camRgb = self.pipeline.create(dai.node.ColorCamera)
            xoutRgb = self.pipeline.create(dai.node.XLinkOut)
            
            xoutRgb.setStreamName("rgb")
            
            # Properties - match working rgb_preview.py settings
            camRgb.setPreviewSize(300, 300)  # Same as rgb_preview.py
            camRgb.setInterleaved(False)
            camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
            
            # Linking
            camRgb.preview.link(xoutRgb.input)
            
            logger.info("Camera pipeline created successfully")
            
        except Exception as e:
            logger.error(f"Failed to create camera pipeline: {e}")
            raise
    
    def _start_camera_thread(self):
        """Start camera capture in background thread"""
        self._camera_thread = threading.Thread(target=self._camera_worker, daemon=True)
        self._camera_thread.start()
    
    def _camera_worker(self):
        """Camera capture worker thread"""
        while not self._should_stop:
            try:
                # Attempt device connection with retry logic
                if not self._connect_device():
                    time.sleep(2.0)  # Wait before retry
                    continue
                
                logger.info("Camera service started successfully")
                
                # Continuous capture loop
                while not self._should_stop and self._connected:
                    try:
                        if self.q_rgb is None:
                            logger.warning("Queue not available, reconnecting...")
                            break
                            
                        # Use blocking get like rgb_preview.py for reliability
                        inRgb = self.q_rgb.get()
                        
                        if inRgb is not None:
                            # Get OpenCV frame
                            frame = inRgb.getCvFrame()
                            
                            # Store latest frame (thread-safe)
                            with self.frame_lock:
                                self.latest_frame = frame.copy()
                            
                            # Reset reconnect attempts on successful frame
                            self._reconnect_attempts = 0
                        
                    except Exception as e:
                        error_msg = str(e)
                        if "X_LINK_ERROR" in error_msg or "Communication exception" in error_msg:
                            logger.error(f"Communication error detected: {e}")
                            self._disconnect_device()
                            break  # Exit inner loop to trigger reconnection
                        else:
                            logger.error(f"Error in camera capture loop: {e}")
                            time.sleep(0.1)  # Brief pause for other errors
                        
            except Exception as e:
                logger.error(f"Camera worker failed: {e}")
                self._disconnect_device()
                
                # Exponential backoff for reconnection
                backoff_time = min(2.0 * (2 ** self._reconnect_attempts), 30.0)
                logger.info(f"Waiting {backoff_time}s before reconnection attempt {self._reconnect_attempts + 1}")
                time.sleep(backoff_time)
                
        logger.info("Camera worker thread stopped")
    
    def is_connected(self) -> bool:
        """Check if camera is connected and capturing"""
        return self._connected and not self._should_stop
    
    def get_latest_frame(self) -> Optional[any]:
        """Get the latest camera frame"""
        if not self.is_connected():
            return None
            
        with self.frame_lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
        return None
    
    def get_thumbnail(self, size: tuple = (150, 150)) -> bytes:
        """Get camera thumbnail as JPEG bytes"""
        frame = self.get_latest_frame()
        if frame is None:
            raise Exception("No camera frame available")
        
        try:
            # Resize to thumbnail size
            thumbnail = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
            
            # Convert to JPEG
            success, buffer = cv2.imencode('.jpg', thumbnail, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not success:
                raise Exception("Failed to encode thumbnail")
            
            return buffer.tobytes()
            
        except Exception as e:
            logger.error(f"Failed to create thumbnail: {e}")
            raise
    
    def get_full_image(self) -> bytes:
        """Get full resolution camera image as JPEG bytes"""
        frame = self.get_latest_frame()
        if frame is None:
            raise Exception("No camera frame available")
        
        try:
            # Convert to JPEG (higher quality for full image)
            success, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not success:
                raise Exception("Failed to encode full image")
            
            return buffer.tobytes()
            
        except Exception as e:
            logger.error(f"Failed to create full image: {e}")
            raise
    
    def capture_for_analysis(self) -> Optional[any]:
        """Capture a frame specifically for AI analysis"""
        frame = self.get_latest_frame()
        if frame is None:
            logger.warning("No frame available for analysis")
            return None
        
        # Return copy for analysis (RGB format for OpenAI)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    def _connect_device(self) -> bool:
        """Connect to DepthAI device with proper error handling"""
        try:
            with self.device_lock:
                if self._reconnect_attempts >= self._max_reconnect_attempts:
                    logger.error(f"Max reconnection attempts ({self._max_reconnect_attempts}) reached")
                    return False
                
                logger.info(f"Connecting to camera (attempt {self._reconnect_attempts + 1})...")
                
                # Create new device connection
                self.device = dai.Device(self.pipeline)
                
                logger.info(f'Connected cameras: {self.device.getConnectedCameraFeatures()}')
                logger.info(f'USB speed: {self.device.getUsbSpeed().name}')
                logger.info(f'Device name: {self.device.getDeviceName()}, Product: {self.device.getProductName()}')
                
                # Create output queue with blocking behavior like rgb_preview.py
                self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=True)
                self._connected = True
                self._reconnect_attempts += 1
                
                return True
                
        except Exception as e:
            logger.error(f"Failed to connect to camera: {e}")
            self._disconnect_device()
            self._reconnect_attempts += 1
            return False
    
    def _disconnect_device(self):
        """Safely disconnect from DepthAI device"""
        with self.device_lock:
            self._connected = False
            self.q_rgb = None
            
            if self.device:
                try:
                    self.device.close()
                except:
                    pass  # Ignore close errors
                self.device = None
                
            logger.info("Device disconnected")
    
    def reset_connection(self):
        """Manually reset camera connection"""
        logger.info("Manually resetting camera connection...")
        self._disconnect_device()
        self._reconnect_attempts = 0
    
    def stop(self):
        """Stop camera service"""
        logger.info("Stopping camera service...")
        
        self._should_stop = True
        self._disconnect_device()
        
        # Wait for camera thread to finish
        if self._camera_thread and self._camera_thread.is_alive():
            self._camera_thread.join(timeout=5.0)
        
        logger.info("Camera service stopped")