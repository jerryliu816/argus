#!/bin/bash

# Robot Control Dashboard Installation Script
# Run this script on the Raspberry Pi to set up the web dashboard

set -e  # Exit on any error

echo "🤖 Installing Robot Control Dashboard..."

# Check if running on Raspberry Pi/Ubuntu
if [[ ! -f /etc/os-release ]]; then
    echo "❌ Cannot determine OS. Please run on Ubuntu/Raspberry Pi."
    exit 1
fi

# Get current directory
DASHBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "📁 Dashboard directory: $DASHBOARD_DIR"

# Check for ROS2 installation
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "❌ ROS2 Humble not found. Please install ROS2 first."
    echo "   Follow: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo "✅ ROS2 Humble found"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Check for Python 3
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found. Installing..."
    sudo apt update
    sudo apt install -y python3 python3-pip python3-venv python3-dev
fi

echo "✅ Python 3 found: $(python3 --version)"

# Ensure we have the necessary Python packages
echo "📦 Installing essential Python packages..."
sudo apt install -y python3-venv python3-pip python3-dev build-essential

# Create Python virtual environment
VENV_DIR="$DASHBOARD_DIR/venv"
if [[ ! -d "$VENV_DIR" ]]; then
    echo "🐍 Creating Python virtual environment..."
    python3 -m venv "$VENV_DIR"
    
    # Check if venv creation was successful
    if [[ ! -f "$VENV_DIR/bin/activate" ]]; then
        echo "❌ Failed to create virtual environment. Installing python3-venv..."
        sudo apt install -y python3-venv python3-dev
        python3 -m venv "$VENV_DIR"
        
        # Final check
        if [[ ! -f "$VENV_DIR/bin/activate" ]]; then
            echo "❌ Still cannot create virtual environment. Trying alternative method..."
            sudo apt install -y python3-virtualenv
            virtualenv -p python3 "$VENV_DIR"
        fi
    fi
fi

# Verify venv exists before activating
if [[ ! -f "$VENV_DIR/bin/activate" ]]; then
    echo "❌ Virtual environment activation script not found!"
    echo "Please check Python installation and try again."
    exit 1
fi

# Activate virtual environment
echo "🐍 Activating virtual environment..."
source "$VENV_DIR/bin/activate"

# Upgrade pip
pip install --upgrade pip

# Install Python dependencies
echo "📦 Installing Python dependencies..."
cd "$DASHBOARD_DIR/backend"
pip install -r requirements.txt

# Install additional system dependencies for DepthAI
echo "📦 Installing system dependencies..."
sudo apt update
sudo apt install -y \
    python3-opencv \
    libusb-1.0-0-dev \
    udev

# Add udev rules for OAK-D camera (if not already present)
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

# Create systemd service file
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
Environment=PATH=$VENV_DIR/bin
ExecStartPre=/bin/bash -c 'source /opt/ros/humble/setup.bash'
ExecStart=$VENV_DIR/bin/python main.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# Set correct permissions
sudo chown $USER:$USER "$DASHBOARD_DIR" -R
sudo chmod +x "$DASHBOARD_DIR/run.sh"

# Enable and start service
echo "🚀 Enabling robot dashboard service..."
sudo systemctl daemon-reload
sudo systemctl enable robot-dashboard.service

# Create desktop shortcut (optional)
DESKTOP_FILE="$HOME/Desktop/Robot-Dashboard.desktop"
if [[ -d "$HOME/Desktop" ]]; then
    echo "🖥️ Creating desktop shortcut..."
    tee "$DESKTOP_FILE" > /dev/null << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Robot Dashboard
Comment=Open Robot Control Dashboard in browser
Exec=xdg-open http://localhost:8000
Icon=applications-internet
Terminal=false
Categories=Network;WebBrowser;
EOF
    chmod +x "$DESKTOP_FILE"
fi

# Get local IP address
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo ""
echo "✅ Installation completed successfully!"
echo ""
echo "🎯 Next steps:"
echo "1. Start the dashboard:"
echo "   sudo systemctl start robot-dashboard.service"
echo ""
echo "2. Check status:"
echo "   sudo systemctl status robot-dashboard.service"
echo ""
echo "3. View logs:"
echo "   sudo journalctl -u robot-dashboard.service -f"
echo ""
echo "4. Access dashboard:"
echo "   Local:  http://localhost:8000"
echo "   Remote: http://$LOCAL_IP:8000"
echo ""
echo "5. To start/stop manually:"
echo "   cd $DASHBOARD_DIR && ./run.sh"
echo ""
echo "📱 Mobile users: Add http://$LOCAL_IP:8000 to your home screen!"
echo ""
echo "🔧 Configuration files:"
echo "   Service: $SERVICE_FILE"
echo "   Dashboard: $DASHBOARD_DIR"
echo ""