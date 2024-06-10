import os
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class DroneImage(Node):
    def __init__(self):
        super().__init__('drone_image')
        self.subscription = self.create_subscription(
            Image, 'image_raw', self.image_upload, 10)

    def image_upload(self, msg):
        self.get_logger().info('Image received')
        # Location and name
        folder_dir = "/project_cf_cam_calibration/drone_images"

        # Location and name
        image_name = 'image_taken.png'

        # Adding image to folder
        with open(os.path.join(folder_dir, image_name), 'wb') as f:
            f.write(msg.data)

        self.get_logger().info('Image uploaded')


def main(args=None):
    rclpy.init(args=args)

    drone_image = DroneImage()

    rclpy.spin(drone_image)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
