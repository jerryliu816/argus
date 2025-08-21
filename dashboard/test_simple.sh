#!/bin/bash

# Test with simplified main.py

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$SCRIPT_DIR/backend"

echo "🤖 Testing Robot Control Dashboard (Simple Version)..."
echo "📁 Backend directory: $BACKEND_DIR"

cd "$BACKEND_DIR"

# Source ROS2 environment
if [[ -f /opt/ros/humble/setup.bash ]]; then
    echo "🔧 Sourcing ROS2 environment..."
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 environment: ROS_DISTRO=$ROS_DISTRO"
else
    echo "⚠️ ROS2 not found - robot control will not work"
fi

# Set ROS environment variables
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo ""
echo "🚀 Starting simplified dashboard server..."
echo "📱 Target: http://192.168.1.201:8000"
echo ""

# Use the simplified main.py
python3 main_simple.py