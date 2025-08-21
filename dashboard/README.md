# Robot Control Dashboard

Web-based real-time control interface for autonomous patrol robot.

## Features

### Phase 1 (Complete)
- ✅ **Real-time robot control** via WebSocket (sub-10ms latency)
- ✅ **Dual input support**: Keyboard (desktop) and touch/joystick (mobile)
- ✅ **Camera preview**: Live thumbnails with click-to-enlarge
- ✅ **Responsive design**: Works on desktop, tablet, and mobile
- ✅ **PWA support**: Can be installed on mobile home screen
- ✅ **Emergency stop**: Immediate movement halt
- ✅ **Speed control**: Adjustable linear and angular speeds
- ✅ **Dock/Undock**: Robot docking station control

### Phase 2 (Future)
- 🔄 **SLAM integration**: Real-time map display
- 🔄 **Nav2 interface**: Point-and-click navigation
- 🔄 **AI analysis**: OpenAI-powered image analysis
- 🔄 **Multi-page interface**: Separate views for different functions

## Technical Approach

### ROS2 Communication Bridge
The dashboard uses a **CLI bridge** approach to communicate with ROS2:
- `cli_bridge.py` executes `ros2 topic pub` commands via subprocess
- This bypasses Python ROS2 node discovery issues 
- Requires `ros2 launch drive controller.launch.py` running first
- Direct CLI commands ensure compatibility with existing ROS2 setup

### Network Configuration
- Server binds to `192.168.1.201:8000` for network access
- WebSocket provides real-time control with <10ms latency
- Camera service directly accesses OAK-D-LITE via DepthAI
- No ROS2 dependencies for camera functionality

## Architecture

```
dashboard/
├── backend/                 # FastAPI server
│   ├── main_simple.py      # Main server (current working version)
│   ├── cli_bridge.py       # ROS2 CLI command bridge
│   ├── camera_service.py   # Direct DepthAI camera access
│   ├── requirements.txt    # Python dependencies
│   └── [legacy files...]   # Multiple debug/test versions
├── frontend/               # Static web interface
│   ├── index.html         # Main dashboard page
│   ├── css/main.css       # Responsive styling
│   ├── js/
│   │   ├── controls.js    # Main controller
│   │   ├── keyboard.js    # Keyboard controls (teleop.py compatible)
│   │   ├── touch.js       # Virtual joystick
│   │   ├── websocket.js   # Real-time communication
│   │   └── camera.js      # Camera display and modal
│   └── manifest.json      # PWA configuration
├── install_simple.sh      # Simple installation script
├── run.sh                 # Manual startup script
└── [debug scripts...]     # Various troubleshooting tools
```

## Installation

### Automated Setup
```bash
cd dashboard
./install_simple.sh
```

### Manual Setup
```bash
# Install dependencies (uses system Python to avoid ROS2 conflicts)
cd backend
pip3 install --user fastapi uvicorn websockets

# Start dashboard (after ROS2 controller is running)
python3 main_simple.py
```

### Prerequisites
**CRITICAL**: Build the ROS2 drive package first:
```bash
cd ~/argus/create3_ws
colcon build --packages-select drive --symlink-install
source install/setup.bash
```

## Usage

### Critical: ROS2 Startup Sequence

**IMPORTANT**: The robot must be properly initialized before the dashboard will work:

```bash
# Terminal 1: Start ROS2 environment and controller (REQUIRED FIRST)
source /opt/ros/humble/setup.bash 
cd ~/argus/create3_ws
source install/setup.bash
ros2 launch drive controller.launch.py

# Terminal 2: Start the dashboard (after controller is running)
cd ~/argus/dashboard
./run.sh
```

### Starting the Dashboard
```bash
# Option 1: Simple startup (after ROS2 controller is running)
cd ~/argus/dashboard/backend
python3 main_simple.py

# Option 2: Using run script
./run.sh

# Option 3: Via systemd service (if configured)
sudo systemctl start robot-dashboard.service
sudo systemctl status robot-dashboard.service
```

### Accessing the Interface
- **Local**: http://localhost:8000
- **Network**: http://192.168.1.201:8000 (configured for network access)
- **Mobile**: Add to home screen for app-like experience

### Controls

#### Keyboard Mode (Desktop)
Same key mappings as the existing `teleop.py`:
- **Movement**: `u,i,o,j,k,l,m,comma,period`
- **Holonomic**: Hold `Shift` + movement keys
- **Speed**: `q/z` (all), `w/x` (linear), `e/c` (angular)
- **Dock**: `d` (dock), `s` (undock)
- **Help**: `h` (toggle help display)

#### Touch Mode (Mobile/Tablet)
- **Virtual joystick**: Drag to move robot
- **Speed sliders**: Adjust linear and angular speeds
- **Touch buttons**: Dock/undock controls
- **Emergency stop**: Large red button

### Camera Features
- **Live thumbnail**: Auto-refreshes every 3 seconds
- **Click to enlarge**: Full-screen modal view
- **Manual refresh**: Force update camera image
- **Download**: Save current image

## Network Requirements

- **Local WiFi**: Robot and control device on same network (192.168.1.x)
- **Bandwidth**: Minimal (control commands only a few bytes)
- **Latency**: Optimized for <10ms response time

## System Requirements

### Raspberry Pi
- Ubuntu 22.04 or newer
- ROS2 Humble
- Python 3.8+
- OAK-D-LITE camera (DepthAI compatible)
- iRobot Create3 with network connectivity

### Control Device
- Modern web browser (2019+ for ES6 support)
- Touch support (mobile/tablet) or keyboard (desktop)
- Network connection to Pi

## Troubleshooting

### Dashboard Won't Start
```bash
# Check service status
sudo systemctl status robot-dashboard.service

# View logs
sudo journalctl -u robot-dashboard.service -f

# Check port availability
netstat -tln | grep :8000
```

### Robot Not Responding

**First, ensure the controller is running properly:**
```bash
# If controller.launch.py fails with joy_dock package error:
cd ~/argus/create3_ws
colcon build --packages-select drive --symlink-install
source install/setup.bash
ros2 launch drive controller.launch.py
```

**Then check ROS2 connectivity:**
1. Verify ROS2 environment: `source /opt/ros/humble/setup.bash`
2. Check Create3 connection: `ros2 topic list`
3. Verify /cmd_vel topic exists: `ros2 topic echo /cmd_vel`
4. Test manual control: `python3 ~/argus/scripts/teleop.py`

### Camera Not Working
1. Check OAK-D connection: `lsusb | grep -i movidius`
2. Test direct access: `python3 ColorCamera/rgb_preview.py`
3. Verify permissions: `ls -la /dev/bus/usb/`

### WebSocket Connection Issues
1. Check firewall: `sudo ufw status`
2. Verify network: `ping [PI_IP_ADDRESS]`
3. Test HTTP access: `curl http://[PI_IP_ADDRESS]:8000/api/status`

## Development

### Adding New Features
1. Backend: Add endpoints to `main.py`
2. Frontend: Create new JS modules in `js/`
3. Update `controls.js` to coordinate new functionality

### Testing
```bash
# Backend testing
cd backend
python -m pytest  # (add tests as needed)

# Frontend testing  
# Open browser developer tools
# Check console for errors
```

## Future Enhancements

### Phase 2: SLAM & Navigation
- File-based SLAM map reading
- Interactive map with robot position
- Point-and-click navigation via Nav2

### Phase 3: AI Integration
- OpenAI API for image analysis
- Object detection with YOLO
- Natural language commands

### Phase 4: Advanced Features
- Multi-robot support
- Video streaming (WebRTC)
- Real-time sensor dashboard
- Autonomous patrol patterns