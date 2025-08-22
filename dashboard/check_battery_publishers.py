#!/usr/bin/env python3

import subprocess
import time

def check_battery_publishers():
    """Check if anyone is publishing to /battery_state"""
    print("=== Checking battery topic publishers ===")
    
    try:
        # Check topic info for publisher count
        result = subprocess.run(['ros2', 'topic', 'info', '/battery_state'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("Battery topic info:")
            print(result.stdout)
            
            # Look for publisher count
            lines = result.stdout.split('\n')
            for line in lines:
                if 'Publisher count:' in line:
                    pub_count = line.split(':')[1].strip()
                    if pub_count == '0':
                        print("❌ No publishers found for /battery_state topic!")
                        return False
                    else:
                        print(f"✅ Found {pub_count} publisher(s)")
                        
        else:
            print("❌ Failed to get topic info:", result.stderr)
            return False
            
        # Check what nodes are running
        print("\n=== Checking running nodes ===")
        result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            print("Running nodes:")
            for node in nodes:
                if node.strip():
                    print(f"  - {node.strip()}")
                    
            # Look for Create3 related nodes
            create3_nodes = [n for n in nodes if 'create3' in n.lower() or 'robot' in n.lower()]
            if create3_nodes:
                print(f"\n✅ Found potential robot nodes: {create3_nodes}")
            else:
                print("\n⚠️ No obvious Create3/robot nodes found")
                
        return True
        
    except Exception as e:
        print(f"❌ Check failed: {e}")
        return False

def check_create3_connectivity():
    """Check if Create3 robot is connected"""
    print("\n=== Checking Create3 connectivity ===")
    
    try:
        # Try to ping the Create3 robot
        result = subprocess.run(['ping', '-c', '1', '192.168.1.68'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("✅ Create3 robot (192.168.1.68) is reachable")
        else:
            print("❌ Cannot reach Create3 robot at 192.168.1.68")
            return False
            
        # Check for any Create3 topics
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            topics = result.stdout.split('\n')
            create3_topics = [t for t in topics if any(keyword in t.lower() for keyword in ['battery', 'dock', 'create', 'robot'])]
            if create3_topics:
                print("✅ Found Create3 related topics:")
                for topic in create3_topics:
                    if topic.strip():
                        print(f"  - {topic.strip()}")
            else:
                print("❌ No Create3 topics found")
                return False
                
        return True
        
    except Exception as e:
        print(f"❌ Connectivity check failed: {e}")
        return False

if __name__ == "__main__":
    print("Battery Publisher Diagnostic")
    print("=" * 40)
    
    pub_ok = check_battery_publishers()
    conn_ok = check_create3_connectivity()
    
    if not pub_ok:
        print("\n🔧 Solution: The Create3 robot may not be connected or the battery publisher is not running.")
        print("   Try:")
        print("   1. Ensure Create3 robot is powered on and connected")
        print("   2. Check if ROS2 discovery is working between robot and this machine")
        print("   3. Verify ROS_DOMAIN_ID matches between devices")