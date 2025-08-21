#!/bin/bash

# Debug script for installation issues

echo "🔍 Debugging Robot Dashboard Installation"
echo "========================================"

# Current directory
echo "📁 Current directory: $(pwd)"
echo "📁 Script directory: $(dirname "$0")"

# Python information
echo ""
echo "🐍 Python Information:"
echo "Python3 location: $(which python3)"
echo "Python3 version: $(python3 --version 2>&1)"
echo "Pip3 location: $(which pip3)"

# Check if python3-venv is available
echo ""
echo "📦 Python venv module:"
python3 -c "import venv; print('✅ venv module available')" 2>/dev/null || echo "❌ venv module not available"

# Check available Python packages
echo ""
echo "📦 Available Python packages:"
dpkg -l | grep python3 | head -10

# Try creating a test venv
echo ""
echo "🧪 Testing venv creation:"
TEST_VENV="/tmp/test_venv_$$"
if python3 -m venv "$TEST_VENV" 2>/dev/null; then
    echo "✅ Test venv created successfully"
    rm -rf "$TEST_VENV"
else
    echo "❌ Test venv creation failed"
    echo "Error output:"
    python3 -m venv "$TEST_VENV" 2>&1 || true
fi

# Check disk space
echo ""
echo "💾 Disk space:"
df -h . | head -2

# Check permissions
echo ""
echo "🔐 Directory permissions:"
ls -la "$(dirname "$0")"

# ROS2 check
echo ""
echo "🤖 ROS2 Information:"
if [[ -f /opt/ros/humble/setup.bash ]]; then
    echo "✅ ROS2 Humble found"
    source /opt/ros/humble/setup.bash
    echo "ROS_DISTRO: $ROS_DISTRO"
else
    echo "❌ ROS2 Humble not found"
fi

# Network check
echo ""
echo "🌐 Network connectivity:"
if ping -c 1 8.8.8.8 >/dev/null 2>&1; then
    echo "✅ Internet connectivity available"
else
    echo "❌ No internet connectivity"
fi

echo ""
echo "🔧 Suggested fixes:"
echo "1. Update package list: sudo apt update"
echo "2. Install Python dev tools: sudo apt install python3-dev python3-venv build-essential"
echo "3. Check available space: df -h"
echo "4. Try manual venv creation: python3 -m venv test_venv"
echo ""