# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
This repo contains software for a Raspberry Pi running Ubuntu 22.04 and ROS2 Humble, connected to an iRobot Create3, OAK-D-LITE depth camera, and a RPLidar A1. This robot is meant to be a home patrol robot, able to map a house with SLAM, navigate with a package like Nav2, and patrol the house autonomously. It can capture images and use OpenAI API to look for anomalies and remember where things are.

## Code Architecture

### ROS2 Workspace Structure
- `create3_ws/`: Main ROS2 workspace containing the drive package
- `create3_ws/src/drive/`: Custom ROS2 package for robot control
  - `drive/joy_dock.py`: Xbox controller node for dock/undock actions (X/Y buttons)
  - `launch/controller.launch.py`: Launch file coordinating joy, teleop_twist_joy, and joy_dock nodes
  - `config/twist.yaml`: Joystick movement parameters and button mappings

### Standalone Scripts
- `scripts/map2img.py`: Subscribes to `/map` topic and saves SLAM maps as timestamped PNG images
- `scripts/capture_image.py`: Interactive RGB camera capture from `/oak/rgb/image_raw` topic
- `scripts/teleop.py`: Enhanced keyboard teleop with dock/undock support (replaces standard teleop_twist_keyboard)
- `ColorCamera/`: Collection of OAK-D-LITE camera utility scripts for various video/image processing

### Network Configuration
- ROS2 FastDDS server: 192.168.1.201
- iRobot Create3: 192.168.1.68
- Files deployed to Raspberry Pi at: `/home/ubuntu`

## Development Commands

### Building the ROS2 Workspace
```bash
cd create3_ws
colcon build --packages-select drive
source install/setup.bash
```

### Testing
- Tests are defined in package.xml using pytest framework
- Run package tests: `colcon test --packages-select drive`

### Robot Operation Commands

#### 1. Manual Driving (Xbox Controller)
Terminal 1:
```bash
source /opt/ros/humble/setup.bash 
cd ~/argus/create3_ws
source install/setup.bash
ros2 launch drive controller.launch.py
```

Terminal 2 (enhanced keyboard control with dock/undock):
```bash
python3 scripts/teleop.py
```

Terminal 2 (standard keyboard alternative):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 2. SLAM Mapping
Terminal 3:
```bash
ros2 launch slam_toolbox online_sync_launch.py
```

Terminal 4 (capture map):
```bash
python3 scripts/map2img.py
```

#### 3. Camera Operations

Terminal 5
```bash
ros2 launch depthai_ros_driver camera.launch.py params_file:="/home/ubuntu/config/my_rgbd.yaml"
```
Terminal 6
```bash
python3 scripts/capture_image.py  # Interactive RGB capture
```

Alternatively that works consistently
```bash
python3 ColorCamera/rgb_preview.py
```

### Key Dependencies
- ROS2 Humble packages: rclpy, sensor_msgs, irobot_create_msgs, nav_msgs, geometry_msgs
- External: cv2, numpy, cv_bridge for image processing
- Hardware: joy package for Xbox controller, teleop_twist_joy for movement

### Enhanced Teleop Controls (scripts/teleop.py)
**Movement**: u,i,o,j,k,l,m,comma,period (same as standard teleop)
**Holonomic**: Hold shift + movement keys  
**Speed**: q/z (all speeds), w/x (linear only), e/c (angular only)
**Dock Commands**: 
- 'd' = dock robot
- 's' = undock robot  
- 'h' = show help
**Stop**: Any other key, Ctrl-C to quit

## Web Dashboard

### Overview
A web-based control interface accessible at `192.168.1.201:8000` providing real-time robot control via browser.

### Starting the Dashboard
**CRITICAL**: Must start ROS2 controller first:
```bash
# Terminal 1: Start ROS2 controller (REQUIRED)
source /opt/ros/humble/setup.bash 
cd ~/argus/create3_ws
source install/setup.bash
ros2 launch drive controller.launch.py

# Terminal 2: Start dashboard (choose one option)
cd ~/argus/dashboard

# Option A: Simplified version (recommended for ROS2 Python issues)
./test_simple.sh

# Option B: Full version with virtual environment
./run.sh

# Option C: Manual startup (alternative)
cd backend && python3 main_simple.py
```

### Dashboard Versions
**main_simple.py** (recommended):
- Uses CLI bridge to bypass ROS2 Python discovery issues
- More robust dependency handling and error recovery
- Better suited for deployment environments

**main.py** (full version):
- Direct ROS2 Python bindings
- Requires proper virtual environment setup
- More features but potentially less stable

### Features
- **Dual input**: Keyboard (desktop) and touch controls (mobile)
- **Real-time control**: WebSocket with <10ms latency
- **Camera preview**: Live OAK-D-LITE thumbnails with click-to-enlarge
- **Mobile PWA**: Can be installed on phone home screen
- **Speed control**: Adjustable linear/angular speeds
- **Dock/undock**: Robot docking station control

### Controls
- **Keyboard**: Same mappings as teleop.py (u,i,o,j,k,l,m,comma,period)
- **Touch**: Virtual joystick for mobile devices
- **Emergency stop**: Immediate movement halt
- **Camera**: Auto-refresh thumbnails, modal view

### Technical Implementation
**main_simple.py** (recommended):
- **Backend**: FastAPI server with WebSocket
- **ROS2 Bridge**: CLI command bridge (cli_bridge.py) - bypasses Python ROS2 discovery issues
- **Camera**: Direct DepthAI access (camera_service.py)
- **Static Files**: Separate CSS/JS/static mounts + PWA manifest support

**main.py** (full version):
- **Backend**: FastAPI server with direct ROS2 Python bindings
- **ROS2 Control**: Direct rclpy integration via RobotController class
- **Dependency**: Requires virtual environment and proper ROS2 Python setup

**Common Features**:
- **Network**: Binds to 192.168.1.201:8000 for remote access
- **WebSocket**: Real-time control with <10ms latency
- **Camera**: Live OAK-D-LITE thumbnails and full-size capture


