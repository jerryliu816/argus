#!/usr/bin/env python3

import sys
import os

# Add backend directory to path
backend_path = os.path.join(os.path.dirname(__file__), 'backend')
sys.path.insert(0, backend_path)

import cli_bridge

def test_battery_monitoring():
    """Test battery monitoring functionality"""
    print("Testing battery monitoring...")
    
    # Get bridge instance
    bridge = cli_bridge.get_bridge()
    
    if bridge.is_connected():
        print("✅ Bridge connected")
        
        # Wait a moment for battery data to be collected
        import time
        print("Waiting 10 seconds for battery data...")
        time.sleep(10)
        
        # Check battery percentage
        battery_percentage = bridge.get_battery_percentage()
        print(f"Battery percentage: {battery_percentage}")
        
        if battery_percentage is not None:
            print(f"✅ Battery monitoring working: {battery_percentage}%")
        else:
            print("❌ Battery data not available yet")
            
    else:
        print("❌ Bridge not connected")
    
    cli_bridge.shutdown()

if __name__ == "__main__":
    test_battery_monitoring()