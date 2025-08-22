#!/usr/bin/env python3

import sys
import os
import subprocess
import time

def test_ros2_battery_access():
    """Test direct ROS2 battery access"""
    print("=== Testing ROS2 battery access ===")
    
    try:
        # Test ros2 topic list
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            topics = result.stdout.split('\n')
            if '/battery_state' in topics:
                print("✅ /battery_state topic found")
            else:
                print("❌ /battery_state topic not found")
                print("Available topics:", [t for t in topics if t.strip()])
                return False
        else:
            print("❌ ros2 topic list failed:", result.stderr)
            return False
            
        # Test battery topic echo
        print("Testing battery topic echo...")
        result = subprocess.run(['ros2', 'topic', 'echo', '/battery_state', '--once'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("✅ Battery topic accessible")
            lines = result.stdout.split('\n')
            for line in lines:
                if 'percentage:' in line:
                    print(f"Battery data: {line.strip()}")
                    return True
            print("❌ No percentage field found in battery data")
            return False
        else:
            print("❌ Battery topic echo failed:", result.stderr)
            return False
            
    except Exception as e:
        print(f"❌ ROS2 test failed: {e}")
        return False

def test_cli_bridge():
    """Test CLI bridge battery monitoring"""
    print("\n=== Testing CLI bridge ===")
    
    try:
        sys.path.insert(0, '/home/ubuntu/argus/dashboard/backend')
        import cli_bridge
        
        bridge = cli_bridge.get_bridge()
        if bridge.is_connected():
            print("✅ CLI bridge connected")
            
            # Wait for battery data
            print("Waiting 5 seconds for battery data...")
            time.sleep(5)
            
            battery = bridge.get_battery_percentage()
            if battery is not None:
                print(f"✅ Battery percentage: {battery}%")
                return True
            else:
                print("❌ Battery percentage is None")
                return False
        else:
            print("❌ CLI bridge not connected")
            return False
            
    except ImportError as e:
        print(f"❌ Failed to import cli_bridge: {e}")
        return False
    except Exception as e:
        print(f"❌ CLI bridge test failed: {e}")
        return False

def test_dashboard_api():
    """Test dashboard API response"""
    print("\n=== Testing Dashboard API ===")
    
    try:
        import urllib.request
        import json
        
        with urllib.request.urlopen('http://192.168.1.201:8000/api/status') as response:
            data = json.loads(response.read())
            
        print("API Response keys:", list(data.keys()))
        
        if 'battery_percentage' in data:
            battery = data['battery_percentage']
            if battery is not None:
                print(f"✅ API returns battery: {battery}%")
                return True
            else:
                print("❌ API battery_percentage is null")
                return False
        else:
            print("❌ No battery_percentage field in API response")
            return False
            
    except Exception as e:
        print(f"❌ API test failed: {e}")
        return False

if __name__ == "__main__":
    print("Battery Status Diagnostic Script")
    print("=" * 40)
    
    ros2_ok = test_ros2_battery_access()
    bridge_ok = test_cli_bridge()
    api_ok = test_dashboard_api()
    
    print(f"\n=== Results ===")
    print(f"ROS2 Access: {'✅' if ros2_ok else '❌'}")
    print(f"CLI Bridge: {'✅' if bridge_ok else '❌'}")
    print(f"Dashboard API: {'✅' if api_ok else '❌'}")
    
    if all([ros2_ok, bridge_ok, api_ok]):
        print("\n🎉 All tests passed! Battery monitoring should work.")
    else:
        print("\n🔧 Some tests failed. Check the errors above.")