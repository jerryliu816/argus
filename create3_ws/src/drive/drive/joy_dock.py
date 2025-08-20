#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rclpy.action import ActionClient
from irobot_create_msgs.action import Dock, Undock

Y_BUTTON = 3  # change if /joy shows a different index for Y
X_BUTTON = 2  # change if /joy shows a different index for X

class JoyDock(Node):
    def __init__(self):
        super().__init__('joy_dock')
        self.sub = self.create_subscription(Joy, '/joy', self.cb, 10)
        self.undock = ActionClient(self, Undock, '/undock')
        self.dock = ActionClient(self, Dock, '/dock')
        self.prev = []

    def cb(self, msg: Joy):
        # edge detect: trigger on press (0->1)
        if len(self.prev) != len(msg.buttons):
            self.prev = [0]*len(msg.buttons)
        if self.edge(msg.buttons, Y_BUTTON):
            self.send(self.undock, Undock)
        if self.edge(msg.buttons, X_BUTTON):
            self.send(self.dock, Dock)
        self.prev = msg.buttons

    def edge(self, buttons, idx):
        try:
            return self.prev[idx] == 0 and buttons[idx] == 1
        except IndexError:
            return False

    def send(self, client, action_type):
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'{action_type.__name__} server not available')
            return
        client.send_goal_async(action_type.Goal())
        self.get_logger().info(f"Sent {action_type.__name__}")

def main():
    rclpy.init()
    rclpy.spin(JoyDock())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

