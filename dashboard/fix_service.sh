#!/bin/bash

# Fix the systemd service configuration

echo "🔧 Fixing robot-dashboard systemd service..."

# Stop the service if running
sudo systemctl stop robot-dashboard.service 2>/dev/null || true

# Get the correct paths
DASHBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$DASHBOARD_DIR/backend"
CURRENT_USER=$(whoami)

echo "📁 Dashboard directory: $DASHBOARD_DIR"
echo "👤 Current user: $CURRENT_USER"

# Determine if we're using venv or simple mode
if [[ -f "$DASHBOARD_DIR/venv/bin/activate" ]]; then
    echo "🐍 Using virtual environment mode"
    PYTHON_PATH="$DASHBOARD_DIR/venv/bin/python"
    USE_VENV=true
else
    echo "🐍 Using system Python mode"
    PYTHON_PATH="/usr/bin/python3"
    USE_VENV=false
fi

# Create a proper systemd service file
SERVICE_FILE="/etc/systemd/system/robot-dashboard.service"
echo "📝 Creating fixed systemd service..."

if [[ "$USE_VENV" == "true" ]]; then
    # Virtual environment version
    sudo tee "$SERVICE_FILE" > /dev/null << EOF
[Unit]
Description=Robot Control Dashboard
After=network.target
Wants=network.target

[Service]
Type=simple
User=$CURRENT_USER
Group=$CURRENT_USER
WorkingDirectory=$BACKEND_DIR
Environment="PATH=$DASHBOARD_DIR/venv/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
Environment="ROS_DISTRO=humble"
Environment="AMENT_PREFIX_PATH=/opt/ros/humble"
Environment="COLCON_PREFIX_PATH=/opt/ros/humble"
Environment="LD_LIBRARY_PATH=/opt/ros/humble/lib"
Environment="PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages"
Environment="CMAKE_PREFIX_PATH=/opt/ros/humble"
Environment="ROS_VERSION=2"
ExecStart=$PYTHON_PATH main.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
KillMode=mixed
KillSignal=SIGTERM

[Install]
WantedBy=multi-user.target
EOF
else
    # System Python version
    sudo tee "$SERVICE_FILE" > /dev/null << EOF
[Unit]
Description=Robot Control Dashboard
After=network.target
Wants=network.target

[Service]
Type=simple
User=$CURRENT_USER
Group=$CURRENT_USER
WorkingDirectory=$BACKEND_DIR
Environment="PATH=/home/$CURRENT_USER/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
Environment="ROS_DISTRO=humble"
Environment="AMENT_PREFIX_PATH=/opt/ros/humble"
Environment="COLCON_PREFIX_PATH=/opt/ros/humble"
Environment="LD_LIBRARY_PATH=/opt/ros/humble/lib"
Environment="PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages"
Environment="CMAKE_PREFIX_PATH=/opt/ros/humble"
Environment="ROS_VERSION=2"
ExecStart=$PYTHON_PATH main.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
KillMode=mixed
KillSignal=SIGTERM

[Install]
WantedBy=multi-user.target
EOF
fi

# Create a startup wrapper script that handles ROS2 sourcing properly
WRAPPER_SCRIPT="$DASHBOARD_DIR/start_dashboard.sh"
echo "📝 Creating startup wrapper script..."

cat > "$WRAPPER_SCRIPT" << EOF
#!/bin/bash

# Dashboard startup wrapper that properly sources ROS2

set -e

# Change to backend directory
cd "$BACKEND_DIR"

# Source ROS2 environment if available
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 environment sourced"
fi

# Activate virtual environment if using venv mode
if [[ "$USE_VENV" == "true" && -f "$DASHBOARD_DIR/venv/bin/activate" ]]; then
    source "$DASHBOARD_DIR/venv/bin/activate"
    echo "✅ Virtual environment activated"
fi

# Set additional environment variables
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Start the dashboard
echo "🚀 Starting Robot Control Dashboard..."
exec $PYTHON_PATH main.py
EOF

chmod +x "$WRAPPER_SCRIPT"

# Update service to use wrapper script
sudo tee "$SERVICE_FILE" > /dev/null << EOF
[Unit]
Description=Robot Control Dashboard
After=network.target
Wants=network.target

[Service]
Type=simple
User=$CURRENT_USER
Group=$CURRENT_USER
WorkingDirectory=$BACKEND_DIR
ExecStart=$WRAPPER_SCRIPT
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
KillMode=mixed
KillSignal=SIGTERM

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd
echo "🔄 Reloading systemd..."
sudo systemctl daemon-reload

# Enable the service
echo "✅ Enabling service..."
sudo systemctl enable robot-dashboard.service

echo ""
echo "✅ Service fixed successfully!"
echo ""
echo "🎯 To test the service:"
echo "   sudo systemctl start robot-dashboard.service"
echo "   sudo systemctl status robot-dashboard.service"
echo ""
echo "🎯 To view logs:"
echo "   sudo journalctl -u robot-dashboard.service -f"
echo ""
echo "🎯 To test manually:"
echo "   $WRAPPER_SCRIPT"
echo ""
EOF