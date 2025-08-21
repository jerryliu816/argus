#!/bin/bash

# Configure the dashboard IP address

set -e

DASHBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_FILE="$DASHBOARD_DIR/backend/main.py"

echo "🔧 Configuring Dashboard IP Address"
echo "===================================="

# Show current network interfaces
echo ""
echo "📡 Available network interfaces:"
ip addr show | grep "inet " | grep -v "127.0.0.1" | awk '{print "   " $2}' | sort

# Get current configuration
CURRENT_IP=$(grep 'target_ip = ' "$BACKEND_FILE" | sed 's/.*target_ip = "\(.*\)".*/\1/')
echo ""
echo "🎯 Current configured IP: $CURRENT_IP"

# Allow user to specify new IP or use default
if [[ -n "$1" ]]; then
    NEW_IP="$1"
    echo "📝 Setting IP to: $NEW_IP (from command line)"
else
    echo ""
    echo "Enter new IP address (or press Enter to keep current):"
    read -r NEW_IP
    
    if [[ -z "$NEW_IP" ]]; then
        NEW_IP="$CURRENT_IP"
        echo "✅ Keeping current IP: $NEW_IP"
    fi
fi

# Validate IP format
if [[ ! "$NEW_IP" =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
    echo "❌ Invalid IP address format: $NEW_IP"
    exit 1
fi

# Update the configuration
echo "🔄 Updating configuration..."
sed -i "s/target_ip = \".*\"/target_ip = \"$NEW_IP\"/" "$BACKEND_FILE"

echo "✅ IP address updated to: $NEW_IP"
echo ""
echo "🎯 Dashboard will be accessible at:"
echo "   http://$NEW_IP:8000"
echo ""
echo "🚀 Restart the dashboard to apply changes:"
echo "   ./test_dashboard.sh"
echo "   or"
echo "   sudo systemctl restart robot-dashboard.service"
echo ""