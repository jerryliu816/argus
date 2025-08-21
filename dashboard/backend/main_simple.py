#!/usr/bin/env python3

# Simplified main.py with better dependency handling

import sys
import os

# Add all possible Python package locations
possible_paths = [
    '/home/ubuntu/.local/lib/python3.10/site-packages',
    '/usr/local/lib/python3.10/dist-packages', 
    '/usr/lib/python3/dist-packages',
    '/usr/lib/python3.10/dist-packages'
]

for path in possible_paths:
    if os.path.exists(path) and path not in sys.path:
        sys.path.insert(0, path)

# Now try imports
try:
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
except ImportError as e:
    print(f"❌ Import error: {e}")
    print("📦 Available packages in Python path:")
    for path in sys.path[:5]:  # Show first 5 paths
        if os.path.exists(path):
            try:
                packages = [d for d in os.listdir(path) if not d.startswith('.')][:10]
                print(f"   {path}: {', '.join(packages[:5])}...")
            except:
                pass
    sys.exit(1)

# Import our custom modules  
try:
    import ros_control_simple as ros_control
    print("✅ ROS control module imported")
except ImportError as e:
    print(f"⚠️ ROS control import failed: {e}")
    ros_control = None

try:
    from camera_service import CameraService
    print("✅ Camera service module imported")
except ImportError as e:
    print(f"⚠️ Camera service import failed: {e}")
    CameraService = None

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
    if ros_control:
        try:
            robot_controller = ros_control.get_robot_controller()
            if robot_controller:
                logger.info("Robot controller initialized")
            else:
                logger.error("Failed to create robot controller")
        except Exception as e:
            logger.error(f"Failed to initialize robot controller: {e}")
    else:
        logger.warning("ROS control module not available")
        
    # Initialize camera service
    if CameraService:
        try:
            camera_service = CameraService()
            logger.info("Camera service initialized")
        except Exception as e:
            logger.error(f"Failed to initialize camera service: {e}")
    else:
        logger.warning("CameraService not available")

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown"""
    global robot_controller, camera_service
    
    logger.info("Shutting down services...")
    
    if ros_control:
        ros_control.shutdown()
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
        if ros_control:
            ros_control.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            ros_control.spin_once()

async def process_control_command(command: Dict):
    """Process incoming control commands"""
    global current_twist, speed_settings
    
    cmd_type = command.get("type")
    logger.info(f"Processing command: {cmd_type}")
    
    if cmd_type == "ping":
        # Ping command - just acknowledge
        logger.debug("Ping received")
        
    elif cmd_type == "move":
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
        if ros_control:
            logger.info(f"Publishing twist: linear=({linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}), angular=({angular_x:.2f}, {angular_y:.2f}, {angular_z:.2f})")
            ros_control.publish_twist(linear_x, linear_y, linear_z, 
                                    angular_x, angular_y, angular_z)
            # Process ROS2 callbacks
            ros_control.spin_once()
        else:
            logger.warning("No robot controller available for movement command")
        
    elif cmd_type == "speed":
        # Speed adjustment
        if "linear" in command:
            speed_settings["linear"] = max(0.1, min(2.0, command["linear"]))
        if "angular" in command:
            speed_settings["angular"] = max(0.1, min(3.0, command["angular"]))
            
        logger.info(f"Speed updated: {speed_settings}")
        
    elif cmd_type == "dock":
        # Dock command
        if ros_control:
            success = ros_control.dock()
            logger.info(f"Dock command result: {success}")
            
    elif cmd_type == "undock":
        # Undock command
        if ros_control:
            success = ros_control.undock()
            logger.info(f"Undock command result: {success}")
            
    elif cmd_type == "stop":
        # Emergency stop
        current_twist = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, 
                        "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        if ros_control:
            ros_control.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            ros_control.spin_once()

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
    return {
        "robot_connected": ros_control is not None and ros_control.is_connected(),
        "camera_connected": camera_service is not None and camera_service.is_connected(),
        "active_connections": len(active_connections),
        "current_speed": speed_settings,
        "current_twist": current_twist
    }

# Serve static files (frontend)
app.mount("/css", StaticFiles(directory="../frontend/css"), name="css")
app.mount("/js", StaticFiles(directory="../frontend/js"), name="js")
app.mount("/static", StaticFiles(directory="../frontend"), name="static")

@app.get("/manifest.json")
async def get_manifest():
    """Serve PWA manifest"""
    return FileResponse("../frontend/manifest.json")

@app.get("/")
async def read_root():
    """Serve main dashboard page"""
    return FileResponse("../frontend/index.html")

if __name__ == "__main__":
    print("🚀 Starting Robot Control Dashboard...")
    print("✅ All imports successful!")
    
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
    
    print(f"📱 Dashboard will be available at: http://{target_ip}:8000")
    
    uvicorn.run(
        "main_simple:app", 
        host=host_ip, 
        port=8000, 
        reload=False,
        log_level="info"
    )