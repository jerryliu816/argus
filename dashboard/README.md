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

## Architecture

```
dashboard/
├── backend/                 # FastAPI server
│   ├── main.py             # Main server with WebSocket
│   ├── ros_control.py      # ROS2 robot control bridge
│   ├── camera_service.py   # Direct DepthAI camera access
│   └── requirements.txt    # Python dependencies
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
├── install.sh             # Automated installation
└── run.sh                 # Manual startup script
```

## Installation

### Automated Setup
```bash
cd dashboard
./install.sh
```

### Manual Setup
```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
cd backend
pip install -r requirements.txt

# Start dashboard
cd ..
./run.sh
```

## Usage

### Starting the Dashboard
```bash
# Via systemd service (recommended)
sudo systemctl start robot-dashboard.service
sudo systemctl status robot-dashboard.service

# Or manually
./run.sh
```

### Accessing the Interface
- **Local**: http://localhost:8000
- **Remote**: http://[PI_IP_ADDRESS]:8000
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
1. Verify ROS2 environment: `source /opt/ros/humble/setup.bash`
2. Check Create3 connection: `ros2 topic list`
3. Test manual control: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

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