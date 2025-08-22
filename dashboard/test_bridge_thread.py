#!/usr/bin/env python3

import sys
import os
import time
import threading

# Add backend to path
sys.path.insert(0, '/home/ubuntu/argus/dashboard/backend')

def test_bridge_threading():
    """Test if the CLI bridge battery thread is working"""
    print("=== Testing CLI bridge battery thread ===")
    
    try:
        import cli_bridge
        
        # Get bridge instance
        bridge = cli_bridge.get_bridge()
        print(f"Bridge connected: {bridge.is_connected()}")
        print(f"Bridge running: {bridge.is_running}")
        
        # Check if battery thread exists and is alive
        print(f"Battery thread exists: {bridge.battery_thread is not None}")
        if bridge.battery_thread:
            print(f"Battery thread alive: {bridge.battery_thread.is_alive()}")
            
        # Check initial battery value
        initial_battery = bridge.get_battery_percentage()
        print(f"Initial battery value: {initial_battery}")
        
        # Force a manual check
        print("\nForcing manual battery check...")
        manual_result = bridge._monitor_battery_once()
        print(f"Manual check result: {manual_result}")
        
        # Check battery value after manual check
        after_manual = bridge.get_battery_percentage()
        print(f"Battery after manual check: {after_manual}")
        
        # Wait and check if thread updates it
        print("\nWaiting 10 seconds to see if background thread updates...")
        time.sleep(10)
        
        final_battery = bridge.get_battery_percentage()
        print(f"Final battery value: {final_battery}")
        
        # Check all running threads
        print(f"\nAll threads ({threading.active_count()}):")
        for thread in threading.enumerate():
            print(f"  - {thread.name}: {thread.is_alive()}")
            
        return final_battery is not None
        
    except Exception as e:
        print(f"❌ Bridge threading test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("CLI Bridge Threading Test")
    print("=" * 40)
    
    success = test_bridge_threading()
    
    print(f"\n=== Result ===")
    if success:
        print("✅ Battery monitoring appears to be working")
    else:
        print("❌ Battery monitoring has issues")