#!/bin/bash

# Quick Python environment check

echo "🐍 Python Environment Debug"
echo "=========================="

echo "Current user: $(whoami)"
echo "Python3 location: $(which python3)"
echo "Pip3 location: $(which pip3)"
echo ""

echo "📦 User-installed packages:"
python3 -m pip list --user | head -10

echo ""
echo "🔍 Looking for FastAPI specifically:"
find /home/$(whoami)/.local -name "*fastapi*" 2>/dev/null | head -5

echo ""
echo "🔍 Python site-packages directories:"
python3 -c "
import site
print('User site packages:', site.USER_SITE)
print('Site packages:', site.getsitepackages())
"

echo ""
echo "🧪 Testing imports with manual path:"
python3 -c "
import sys
import os
user = os.environ.get('USER', 'ubuntu')
user_site = f'/home/{user}/.local/lib/python3.10/site-packages'
if user_site not in sys.path:
    sys.path.insert(0, user_site)

try:
    import fastapi
    print('✅ FastAPI import successful')
except Exception as e:
    print('❌ FastAPI import failed:', e)
"

echo ""
echo "🔧 Current PYTHONPATH:"
echo "PYTHONPATH=$PYTHONPATH"
echo ""
echo "PATH=$PATH"