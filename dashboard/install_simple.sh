#!/bin/bash

# Simplified Robot Control Dashboard Installation Script
# Use this if the main install.sh fails with venv issues

set -e

echo "🤖 Installing Robot Control Dashboard (Simple Mode)..."

# Get current directory
DASHBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "📁 Dashboard directory: $DASHBOARD_DIR"

# Check for ROS2 installation
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "❌ ROS2 Humble not found. Please install ROS2 first."
    exit 1
fi

echo "✅ ROS2 Humble found"

# Update package list
echo "📦 Updating package list..."
sudo apt update

# Install Python and dependencies
echo "📦 Installing Python dependencies..."
sudo apt install -y \
    python3 \
    python3-pip \
    python3-dev \
    build-essential \
    python3-opencv \
    libusb-1.0-0-dev \
    udev

# Install Python packages system-wide (not recommended for production, but works for testing)
echo "📦 Installing Python packages..."
pip3 install --user \
    fastapi==0.104.1 \
    "uvicorn[standard]==0.24.0" \
    websockets==12.0 \
    depthai==2.23.0.0 \
    "opencv-python==4.8.1.78" \
    "numpy==1.24.3"

# Add udev rules for OAK-D camera
UDEV_RULE="/etc/udev/rules.d/80-movidius.rules"
if [[ ! -f "$UDEV_RULE" ]]; then
    echo "📷 Adding udev rules for OAK-D camera..."
    sudo tee "$UDEV_RULE" > /dev/null << 'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="2485", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="f63b", MODE="0666"
EOF
    sudo udevadm control --reload-rules
    sudo udevadm trigger
fi

# Create systemd service file for simple mode
SERVICE_FILE="/etc/systemd/system/robot-dashboard.service"
echo "🔧 Creating systemd service..."
sudo tee "$SERVICE_FILE" > /dev/null << EOF
[Unit]
Description=Robot Control Dashboard
After=network.target
Wants=network.target

[Service]
Type=simple
User=$USER
Group=$USER
WorkingDirectory=$DASHBOARD_DIR/backend
Environment=PATH=/home/$USER/.local/bin:\$PATH
ExecStartPre=/bin/bash -c 'source /opt/ros/humble/setup.bash'
ExecStart=/usr/bin/python3 main.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# Create a simple run script for this mode
cat > "$DASHBOARD_DIR/run_simple.sh" << 'EOF'
#!/bin/bash

# Simple run script without virtual environment

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$SCRIPT_DIR/backend"

echo "🤖 Starting Robot Control Dashboard (Simple Mode)..."

# Source ROS2 environment
if [[ -f /opt/ros/humble/setup.bash ]]; then
    echo "🔧 Sourcing ROS2 environment..."
    source /opt/ros/humble/setup.bash
fi

# Change to backend directory
cd "$BACKEND_DIR"

# Get local IP
LOCAL_IP=$(hostname -I | awk '{print $1}' 2>/dev/null || echo "localhost")

echo ""
echo "🚀 Starting dashboard server..."
echo "📱 Access URLs:"
echo "   Local:  http://localhost:8000"
echo "   Remote: http://$LOCAL_IP:8000"
echo ""

# Start with system Python
python3 main.py
EOF

chmod +x "$DASHBOARD_DIR/run_simple.sh"

# Set permissions
sudo chown $USER:$USER "$DASHBOARD_DIR" -R

# Enable service
echo "🚀 Enabling robot dashboard service..."
sudo systemctl daemon-reload
sudo systemctl enable robot-dashboard.service

# Get local IP
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo ""
echo "✅ Simple installation completed!"
echo ""
echo "🎯 To start the dashboard:"
echo "   sudo systemctl start robot-dashboard.service"
echo ""
echo "🎯 Or run manually:"
echo "   $DASHBOARD_DIR/run_simple.sh"
echo ""
echo "📱 Access at: http://$LOCAL_IP:8000"
echo ""
echo "⚠️  Note: This installation uses system Python packages."
echo "   For production, consider using the full install.sh with virtual environment."
echo ""