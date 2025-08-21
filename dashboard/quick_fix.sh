#!/bin/bash

# Quick fix for Python dependency issues

echo "🔧 Quick fix for Python dependencies..."

# Install click manually
echo "📦 Installing click..."
python3 -m pip install --user click

# Install all requirements again to make sure everything is there
echo "📦 Installing all requirements..."
python3 -m pip install --user -r backend/requirements.txt

# Create a simple test to see what's working
echo "🧪 Testing imports..."
python3 -c "
import sys
import os
sys.path.insert(0, '/home/ubuntu/.local/lib/python3.10/site-packages')

try:
    import click
    print('✅ Click:', click.__version__)
except Exception as e:
    print('❌ Click:', e)

try:
    import uvicorn
    print('✅ Uvicorn:', uvicorn.__version__)
except Exception as e:
    print('❌ Uvicorn:', e)

try:
    import fastapi
    print('✅ FastAPI:', fastapi.__version__)
except Exception as e:
    print('❌ FastAPI:', e)
"

echo "✅ Fix complete!"