#!/bin/bash

# Fix Python dependencies for dashboard

set -e

echo "🔧 Fixing Python Dependencies"
echo "============================="

# Check current user
CURRENT_USER=$(whoami)
echo "👤 Current user: $CURRENT_USER"

# Check what Python packages are installed
echo ""
echo "📦 Checking installed packages..."
echo "Pip3 location: $(which pip3)"
echo "Python3 location: $(which python3)"

# Check if packages are installed
echo ""
echo "🔍 Checking package locations..."
python3 -m pip list --user | grep -E "(fastapi|uvicorn|depthai)" || echo "❌ No user packages found"

# Try installing dependencies again with explicit user flag
echo ""
echo "📥 Installing/upgrading dependencies..."
python3 -m pip install --user --upgrade \
    fastapi==0.104.1 \
    "uvicorn[standard]==0.24.0" \
    websockets==12.0 \
    depthai==2.23.0.0 \
    "opencv-python==4.8.1.78" \
    "numpy==1.24.3"

# Verify installation
echo ""
echo "✅ Verifying installation..."
python3 -c "
import sys
sys.path.insert(0, '/home/$CURRENT_USER/.local/lib/python3.10/site-packages')
try:
    import fastapi
    print('✅ FastAPI: ', fastapi.__version__)
except ImportError as e:
    print('❌ FastAPI: ', e)

try:
    import uvicorn
    print('✅ Uvicorn: ', uvicorn.__version__)
except ImportError as e:
    print('❌ Uvicorn: ', e)

try:
    import depthai
    print('✅ DepthAI: ', depthai.__version__)
except ImportError as e:
    print('❌ DepthAI: ', e)

try:
    import rclpy
    print('✅ ROS2 rclpy: Available')
except ImportError as e:
    print('❌ ROS2 rclpy: ', e)
"

# Show Python path info
echo ""
echo "🐍 Python path information:"
python3 -c "
import sys
print('Python executable:', sys.executable)
print('Python version:', sys.version)
print('Python paths:')
for p in sys.path:
    if p:
        print('  ', p)
"

echo ""
echo "✅ Dependencies check complete!"
echo ""
echo "🎯 Next steps:"
echo "1. Try running the dashboard again:"
echo "   ./test_dashboard.sh"
echo ""
echo "2. If still having issues, try creating a virtual environment:"
echo "   python3 -m venv venv"
echo "   source venv/bin/activate"
echo "   pip install -r backend/requirements.txt"
echo ""