import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from datetime import datetime
import os

class MapToImage(Node):
    def __init__(self):
        super().__init__('map_to_image')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            20
        )
        self.output_dir = '/tmp/slam_maps'
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info("Subscribed to /map")

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Convert to grayscale base image
        img = np.zeros_like(data, dtype=np.uint8)
        img[data == -1] = 127  # Unknown → gray
        img[data == 0] = 255   # Free → white
        img[data == 100] = 0   # Occupied → black

        # Flip vertically for correct orientation
        img = np.flipud(img)

        # Apply color map (optional, gives heatmap style)
        color_img = cv2.applyColorMap(img, cv2.COLORMAP_JET)

        # Scale up image (2x resolution here, change as needed)
        upscale_factor = 4
        large_img = cv2.resize(color_img, (width * upscale_factor, height * upscale_factor), interpolation=cv2.INTER_NEAREST)

        # Generate timestamped filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.output_dir, f"map_{timestamp}.png")

        # Save image
        cv2.imwrite(filename, large_img)
        self.get_logger().info(f"Saved map to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = MapToImage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

