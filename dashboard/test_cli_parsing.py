#!/usr/bin/env python3

import subprocess
import os

def test_battery_parsing():
    """Test the exact battery parsing logic from CLI bridge"""
    print("=== Testing CLI bridge battery parsing ===")
    
    # Get ROS environment (same as CLI bridge)
    env = os.environ.copy()
    env.update({
        'ROS_DOMAIN_ID': '0',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'ROS_DISCOVERY_SERVER': '127.0.0.1:11811',
        'ROS_LOCALHOST_ONLY': '0'
    })
    
    try:
        # Use the same command as CLI bridge
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '/battery_state', '--once'],
            capture_output=True,
            text=True,
            timeout=10,
            env=env
        )
        
        if result.returncode == 0 and result.stdout:
            print("✅ Got battery data via CLI")
            print(f"Raw output length: {len(result.stdout)} characters")
            
            # Parse the battery percentage (same logic as CLI bridge)
            lines = result.stdout.split('\n')
            percentage_found = False
            
            for i, line in enumerate(lines):
                print(f"Line {i}: '{line.strip()}'")
                if line.strip().startswith('percentage:'):
                    percentage_found = True
                    try:
                        percentage_str = line.split(':')[1].strip()
                        percentage = float(percentage_str)
                        # Convert from 0-1 range to 0-100 percentage
                        battery_percentage = int(percentage * 100)
                        print(f"✅ Parsed percentage: {percentage_str} -> {percentage} -> {battery_percentage}%")
                        return battery_percentage
                    except (ValueError, IndexError) as e:
                        print(f"❌ Failed to parse percentage: {e}")
                        print(f"   Line content: '{line}'")
                        print(f"   Split result: {line.split(':')}")
                        return None
            
            if not percentage_found:
                print("❌ No 'percentage:' line found in output")
                print("First 10 lines of output:")
                for i, line in enumerate(lines[:10]):
                    print(f"  {i}: '{line}'")
                return None
                
        else:
            print(f"❌ Command failed with return code {result.returncode}")
            print(f"stderr: {result.stderr}")
            return None
            
    except subprocess.TimeoutExpired:
        print("❌ Command timed out")
        return None
    except Exception as e:
        print(f"❌ Parsing test failed: {e}")
        return None

def test_current_cli_bridge():
    """Test the actual CLI bridge implementation"""
    print("\n=== Testing actual CLI bridge ===")
    
    try:
        import sys
        sys.path.insert(0, '/home/ubuntu/argus/dashboard/backend')
        import cli_bridge
        
        # Get bridge and force a fresh battery check
        bridge = cli_bridge.get_bridge()
        print(f"Bridge connected: {bridge.is_connected()}")
        
        # Manually trigger battery monitoring once
        print("Running manual battery check...")
        bridge._monitor_battery_once()  # We'll add this method
        
        battery = bridge.get_battery_percentage()
        print(f"Battery result: {battery}")
        
        return battery
        
    except Exception as e:
        print(f"❌ CLI bridge test failed: {e}")
        return None

if __name__ == "__main__":
    print("CLI Battery Parsing Test")
    print("=" * 40)
    
    parsed_battery = test_battery_parsing()
    # bridge_battery = test_current_cli_bridge()
    
    print(f"\n=== Results ===")
    print(f"Direct parsing: {parsed_battery}%")
    # print(f"CLI bridge: {bridge_battery}%")