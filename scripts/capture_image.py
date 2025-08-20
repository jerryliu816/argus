#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime
import threading

class InteractiveCapture(Node):
    def __init__(self):
        super().__init__('interactive_capture')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_seq = None
        self.new_image = None
        self.image_event = threading.Event()

        self.sub = self.create_subscription(Image, '/oak/rgb/image_raw', self.image_callback, 10)
        threading.Thread(target=self.input_loop, daemon=True).start()

    def image_callback(self, msg):
        with self.lock:
            # Only trigger event if it's a new image (different sequence number)
            if msg.header.seq != self.latest_seq:
                self.latest_seq = msg.header.seq
                self.new_image = msg
                self.image_event.set()

    def input_loop(self):
        while rclpy.ok():
            input("📸 Press Enter to capture a fresh image... ")

            # Clear event and wait for next unique image
            self.image_event.clear()
            print("⌛ Waiting for next image...")
            if self.image_event.wait(timeout=3.0):
                with self.lock:
                    self.save_image(self.new_image)
            else:
                print("⚠️ Timeout: no new image received. Check if camera is still publishing.")

    def save_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'image_{timestamp}.jpg'
        cv2.imwrite(filename, img)
        self.get_logger().info(f"✅ Saved {filename}")

def main():
    rclpy.init()
    node = InteractiveCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

