import os
import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from sensor_msgs.msg import Image


class DroneImage(Node):
    def __init__(self):
        super().__init__('drone_image')
        self.subscription = self.create_subscription(
            Image, 'drone0/sensor_measurements/cam/image_raw', self.image_upload, 10)
        self.i = 0

    def image_upload(self, msg):
        self.get_logger().info('Image received')
        # Location and name
        folder_dir = "drone_images"

        # Conerting ROS images to compatible file
        image_np = np.frombuffer(msg.data, dtype=np.uint8)

        image_np = image_np.reshape((msg.height, msg.width, -1))

        # Location and name
        image_name = f'image_taken{self.i}.png'
        image_path = os.path.join(folder_dir, image_name)

        # Adding image to folder
        cv2.imwrite(image_path, image_np)

        self.get_logger().info('Image uploaded')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    drone_image = DroneImage()

    rclpy.spin(drone_image)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
