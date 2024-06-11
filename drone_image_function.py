import os
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class DroneImage(Node):
    def __init__(self):
        super().__init__('drone_image')
        self.subscription = self.create_subscription(
            Image, 'drone0/sensor_measurements/cam/image_raw', self.image_upload, 10)
        self.i = 0

        # self.folder_dir = "/project_cf_cam_calibration/drone_images"

        # os.makedirs(self.folder_dir, exist_ok=True)

    def image_upload(self, msg):
        self.get_logger().info('Image received')
        # Location and name
        # you direct it to drone_images becasue it is already launching from project_cf_cam_calibration
        folder_dir = "drone_images"

        # Location and name
        image_name = f'image_taken{self.i}.png'

        # Adding image to folder
        with open(os.path.join(folder_dir, image_name), 'wb') as f:
            f.write(msg.data)

        self.get_logger().info('Image uploaded')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    drone_image = DroneImage()

    rclpy.spin(drone_image)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
