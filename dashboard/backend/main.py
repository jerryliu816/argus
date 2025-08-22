#!/usr/bin/env python3

# Fix Python path to include user packages before importing
import sys
import os
import site

# Add user site-packages to Python path (needed when ROS2 overrides PYTHONPATH)
user_site = site.getusersitepackages()
if user_site not in sys.path:
    sys.path.insert(0, user_site)

# Also add the specific user packages directory for this user
current_user = os.environ.get('USER', 'ubuntu')
user_packages = f'/home/{current_user}/.local/lib/python3.10/site-packages'
if user_packages not in sys.path:
    sys.path.insert(0, user_packages)

import asyncio
import json
import logging
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, Response
import uvicorn
from typing import Dict, Set
import threading
import time

# Import our custom modules
from ros_control import RobotController
from camera_service import CameraService

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(title="Robot Control Dashboard", version="1.0.0")

# Global instances
robot_controller = None
camera_service = None
active_connections: Set[WebSocket] = set()

# Control state
current_twist = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, 
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
speed_settings = {"linear": 0.5, "angular": 1.0}

@app.on_event("startup")
async def startup_event():
    """Initialize services on startup"""
    global robot_controller, camera_service
    
    logger.info("Starting Robot Control Dashboard...")
    
    # Initialize robot controller
    try:
        robot_controller = RobotController()
        logger.info("Robot controller initialized")
    except Exception as e:
        logger.error(f"Failed to initialize robot controller: {e}")
        
    # Initialize camera service
    try:
        camera_service = CameraService()
        logger.info("Camera service initialized")
    except Exception as e:
        logger.error(f"Failed to initialize camera service: {e}")

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown"""
    global robot_controller, camera_service
    
    logger.info("Shutting down services...")
    
    if robot_controller:
        robot_controller.stop()
    if camera_service:
        camera_service.stop()

# WebSocket endpoint for real-time control
@app.websocket("/ws/control")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.add(websocket)
    logger.info(f"Client connected. Active connections: {len(active_connections)}")
    
    try:
        while True:
            # Receive command from client
            data = await websocket.receive_text()
            command = json.loads(data)
            
            # Process command
            await process_control_command(command)
            
            # Echo back for confirmation
            await websocket.send_text(json.dumps({
                "type": "ack",
                "command": command,
                "timestamp": time.time()
            }))
            
    except WebSocketDisconnect:
        active_connections.remove(websocket)
        logger.info(f"Client disconnected. Active connections: {len(active_connections)}")
        
        # Send stop command when client disconnects
        if robot_controller:
            robot_controller.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

async def process_control_command(command: Dict):
    """Process incoming control commands"""
    global current_twist, speed_settings
    
    cmd_type = command.get("type")
    
    if cmd_type == "move":
        # Movement command
        linear_x = command.get("linear_x", 0.0)
        linear_y = command.get("linear_y", 0.0) 
        linear_z = command.get("linear_z", 0.0)
        angular_x = command.get("angular_x", 0.0)
        angular_y = command.get("angular_y", 0.0)
        angular_z = command.get("angular_z", 0.0)
        
        # Apply speed scaling
        linear_x *= speed_settings["linear"]
        linear_y *= speed_settings["linear"]
        linear_z *= speed_settings["linear"]
        angular_z *= speed_settings["angular"]
        
        # Update current twist
        current_twist = {
            "linear": {"x": linear_x, "y": linear_y, "z": linear_z},
            "angular": {"x": angular_x, "y": angular_y, "z": angular_z}
        }
        
        # Send to robot
        if robot_controller:
            robot_controller.publish_twist(linear_x, linear_y, linear_z, 
                                         angular_x, angular_y, angular_z)
        
    elif cmd_type == "speed":
        # Speed adjustment
        if "linear" in command:
            speed_settings["linear"] = max(0.1, min(2.0, command["linear"]))
        if "angular" in command:
            speed_settings["angular"] = max(0.1, min(3.0, command["angular"]))
            
        logger.info(f"Speed updated: {speed_settings}")
        
    elif cmd_type == "dock":
        # Dock command
        if robot_controller:
            success = robot_controller.dock()
            logger.info(f"Dock command result: {success}")
            
    elif cmd_type == "undock":
        # Undock command
        if robot_controller:
            success = robot_controller.undock()
            logger.info(f"Undock command result: {success}")
            
    elif cmd_type == "stop":
        # Emergency stop
        current_twist = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, 
                        "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        if robot_controller:
            robot_controller.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

# Camera endpoints
@app.get("/api/camera/thumbnail")
async def get_camera_thumbnail():
    """Get camera thumbnail image"""
    if not camera_service:
        raise HTTPException(status_code=503, detail="Camera service not available")
    
    try:
        image_data = camera_service.get_thumbnail()
        return Response(content=image_data, media_type="image/jpeg")
    except Exception as e:
        logger.error(f"Failed to get thumbnail: {e}")
        raise HTTPException(status_code=500, detail="Failed to capture image")

@app.get("/api/camera/full")
async def get_camera_full():
    """Get full resolution camera image"""
    if not camera_service:
        raise HTTPException(status_code=503, detail="Camera service not available")
    
    try:
        image_data = camera_service.get_full_image()
        return Response(content=image_data, media_type="image/jpeg")
    except Exception as e:
        logger.error(f"Failed to get full image: {e}")
        raise HTTPException(status_code=500, detail="Failed to capture image")

# Status endpoint
@app.get("/api/status")
async def get_status():
    """Get system status"""
    battery_percentage = None
    battery_charging = None
    is_docked = None
    dock_visible = None
    if robot_controller and robot_controller.is_connected():
        battery_percentage = robot_controller.get_battery_percentage()
        battery_charging = robot_controller.get_battery_charging()
        is_docked = robot_controller.get_is_docked()
        dock_visible = robot_controller.get_dock_visible()
    
    return {
        "robot_connected": robot_controller is not None and robot_controller.is_connected(),
        "camera_connected": camera_service is not None and camera_service.is_connected(),
        "active_connections": len(active_connections),
        "current_speed": speed_settings,
        "current_twist": current_twist,
        "battery_percentage": battery_percentage,
        "battery_charging": battery_charging,
        "is_docked": is_docked,
        "dock_visible": dock_visible
    }

# Serve static files (frontend)
app.mount("/static", StaticFiles(directory="../frontend"), name="static")

@app.get("/")
async def read_root():
    """Serve main dashboard page"""
    return FileResponse("../frontend/index.html")

if __name__ == "__main__":
    # Configure server to bind to specific IP
    import socket
    
    # Try to get the 192.168.1.x IP address
    target_ip = "192.168.1.201"
    
    # Verify the IP is available on this machine
    try:
        # Test if we can bind to this IP
        test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        test_socket.bind((target_ip, 0))  # Bind to any available port for testing
        test_socket.close()
        host_ip = target_ip
        logger.info(f"Binding to target IP: {host_ip}")
    except OSError:
        # Fall back to all interfaces if specific IP not available
        host_ip = "0.0.0.0"
        logger.warning(f"Could not bind to {target_ip}, using {host_ip} instead")
    
    uvicorn.run(
        "main:app", 
        host=host_ip, 
        port=8000, 
        reload=False,  # Disable reload for production
        log_level="info"
    )