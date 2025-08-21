#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Dock, Undock

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
 u i o
 j k l
 m , .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
 U I O
 J K L
 M < >

t : up (+z)
b : down (-z)

Dock/Undock controls:
---------------------------
d : dock robot
s : undock robot

Speed controls:
---------------------------
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

anything else : stop

h : show this help message
CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
 }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
}

dockBindings = {
    'd': 'dock',
    's': 'undock'
}

class TeleopTwistKeyboardDock(Node):

    def __init__(self):
        super().__init__('teleop_twist_keyboard_dock')
        
        # Twist publisher
        self.pub = self.create_publisher(Twist, 'cmd_vel', qos_profile_default)
        
        # Dock/Undock action clients
        self.dock_client = ActionClient(self, Dock, '/dock')
        self.undock_client = ActionClient(self, Undock, '/undock')
        
        # Movement parameters
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0
        
        # Dock command tracking (prevent spam)
        self.last_dock_command = None

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

    def send_dock_command(self, command_type):
        """Send dock or undock command with proper error handling"""
        if command_type == 'dock':
            client = self.dock_client
            action_type = Dock
            action_name = "Dock"
        else:  # undock
            client = self.undock_client
            action_type = Undock
            action_name = "Undock"
        
        # Prevent sending same command repeatedly
        if self.last_dock_command == command_type:
            self.get_logger().info(f"{action_name} command already sent, ignoring repeat")
            return
            
        # Check if action server is available
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'{action_name} server not available')
            return
        
        # Send the goal
        goal = action_type.Goal()
        client.send_goal_async(goal)
        self.get_logger().info(f"Sent {action_name} command")
        self.last_dock_command = command_type

    def run(self):
        print(msg)
        print(self.vels(self.speed, self.turn))
        
        try:
            while True:
                key = self.getKey()
                
                # Handle movement keys
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.y = moveBindings[key][1]
                    self.z = moveBindings[key][2]
                    self.th = moveBindings[key][3]
                    
                # Handle speed adjustment keys
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    
                    print(self.vels(self.speed, self.turn))
                    if (self.status == 14):
                        print(msg)
                    self.status = (self.status + 1) % 15
                    
                # Handle dock/undock keys
                elif key in dockBindings.keys():
                    command = dockBindings[key]
                    self.send_dock_command(command)
                    
                # Handle help key
                elif key == 'h':
                    print(msg)
                    print(self.vels(self.speed, self.turn))
                    
                # Stop movement on any other key
                else:
                    self.x = 0.0
                    self.y = 0.0
                    self.z = 0.0
                    self.th = 0.0
                    self.last_dock_command = None  # Reset dock command tracking
                    if (key == '\x03'):  # Ctrl-C
                        break

                # Publish twist message
                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.th * self.turn
                self.pub.publish(twist)

        except Exception as e:
            print(e)

        finally:
            # Send stop command before exiting
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main():
    rclpy.init()
    
    teleop_node = TeleopTwistKeyboardDock()
    
    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()