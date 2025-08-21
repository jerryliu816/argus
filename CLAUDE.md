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


