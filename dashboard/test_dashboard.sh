#!/bin/bash

# Quick test script to start dashboard manually (bypasses systemd issues)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$SCRIPT_DIR/backend"

echo "🤖 Testing Robot Control Dashboard manually..."
echo "📁 Backend directory: $BACKEND_DIR"

# Check if backend directory exists
if [[ ! -d "$BACKEND_DIR" ]]; then
    echo "❌ Backend directory not found: $BACKEND_DIR"
    exit 1
fi

# Check if main.py exists
if [[ ! -f "$BACKEND_DIR/main.py" ]]; then
    echo "❌ main.py not found in backend directory"
    exit 1
fi

# Change to backend directory
cd "$BACKEND_DIR"

# Source ROS2 environment
if [[ -f /opt/ros/humble/setup.bash ]]; then
    echo "🔧 Sourcing ROS2 environment..."
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 environment: ROS_DISTRO=$ROS_DISTRO"
else
    echo "⚠️ ROS2 not found - robot control will not work"
fi

# Check Python environment
if [[ -f "../venv/bin/activate" ]]; then
    echo "🐍 Activating virtual environment..."
    source "../venv/bin/activate"
    echo "✅ Using virtual environment Python: $(which python)"
else
    echo "🐍 Using system Python: $(which python3)"
    # Add user-installed packages to Python path
    export PYTHONPATH="/home/$USER/.local/lib/python3.10/site-packages:$PYTHONPATH"
    export PATH="/home/$USER/.local/bin:$PATH"
    echo "✅ Added user packages to Python path"
fi

# Set ROS environment variables
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Check Python dependencies
echo "📦 Checking Python dependencies..."
python3 -c "import fastapi; print('✅ FastAPI available')" 2>/dev/null || echo "❌ FastAPI not available"
python3 -c "import uvicorn; print('✅ Uvicorn available')" 2>/dev/null || echo "❌ Uvicorn not available"
python3 -c "import rclpy; print('✅ ROS2 rclpy available')" 2>/dev/null || echo "❌ ROS2 rclpy not available"
python3 -c "import depthai; print('✅ DepthAI available')" 2>/dev/null || echo "❌ DepthAI not available"

# Check if port 8000 is free
if netstat -tln 2>/dev/null | grep -q ":8000 "; then
    echo "⚠️ Port 8000 is already in use. Stopping any running services..."
    sudo systemctl stop robot-dashboard.service 2>/dev/null || true
    sleep 2
fi

# Get network info - prioritize 192.168.1.x subnet
LOCAL_IP=$(hostname -I | tr ' ' '\n' | grep '^192\.168\.1\.' | head -1)
if [[ -z "$LOCAL_IP" ]]; then
    LOCAL_IP=$(hostname -I | awk '{print $1}' 2>/dev/null || echo "localhost")
fi

echo ""
echo "🚀 Starting dashboard server..."
echo "📱 Access URLs:"
echo "   Target:  http://192.168.1.201:8000"
echo "   Local:   http://localhost:8000"
echo "   Auto:    http://$LOCAL_IP:8000"
echo ""
echo "🔧 Press Ctrl+C to stop"
echo "📊 Watch this terminal for error messages"
echo ""

# Start the server
if [[ -f "../venv/bin/python" ]]; then
    ../venv/bin/python main.py
else
    python3 main.py
fi