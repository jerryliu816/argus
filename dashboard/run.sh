#!/bin/bash

# Robot Control Dashboard Startup Script
# Use this to manually start the dashboard (alternative to systemd service)

set -e

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$SCRIPT_DIR/backend"
VENV_DIR="$SCRIPT_DIR/venv"

echo "🤖 Starting Robot Control Dashboard..."

# Check if virtual environment exists
if [[ ! -d "$VENV_DIR" ]]; then
    echo "❌ Virtual environment not found. Please run install.sh first."
    exit 1
fi

# Check if we're in the right directory
if [[ ! -f "$BACKEND_DIR/main.py" ]]; then
    echo "❌ Backend files not found. Please run from dashboard directory."
    exit 1
fi

# Source ROS2 environment
if [[ -f /opt/ros/humble/setup.bash ]]; then
    echo "🔧 Sourcing ROS2 environment..."
    source /opt/ros/humble/setup.bash
else
    echo "⚠️ ROS2 not found - robot control will not work"
fi

# Activate virtual environment
echo "🐍 Activating Python environment..."
source "$VENV_DIR/bin/activate"

# Change to backend directory
cd "$BACKEND_DIR"

# Check if port 8000 is available
if netstat -tln | grep -q ":8000 "; then
    echo "⚠️ Port 8000 is already in use. Stopping existing service..."
    sudo systemctl stop robot-dashboard.service 2>/dev/null || true
    sleep 2
fi

# Get local IP for display
LOCAL_IP=$(hostname -I | awk '{print $1}' 2>/dev/null || echo "localhost")

echo ""
echo "🚀 Starting dashboard server..."
echo "📱 Access URLs:"
echo "   Local:  http://localhost:8000"
echo "   Remote: http://$LOCAL_IP:8000"
echo ""
echo "🔧 Controls:"
echo "   Ctrl+C to stop"
echo "   View logs in terminal"
echo ""

# Start the server
python main.py