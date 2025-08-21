#!/bin/bash

# Install missing Python dependencies

echo "🔧 Installing missing Python dependencies..."

# Install the missing click package and other uvicorn dependencies
python3 -m pip install --user \
    click \
    typing-extensions \
    pydantic \
    starlette \
    python-multipart \
    uvloop \
    httptools \
    watchfiles

echo "✅ Dependencies installed!"
echo ""
echo "🎯 Now try running the dashboard again:"
echo "   ./test_dashboard.sh"