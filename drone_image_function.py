import os
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class DroneImage(Node):
    def __init__(self):
        super().__init__('drone_image')
        self.publisher = self.create_publisher(Image, 'charruco_image', 10)
        self.i = 0

    def image_upload(self):
        # Obtaining image
        msg = Image()
        msg.data = 'image here'

        # Location and name
        self.folder_dir = "/project_cf_cam_calibration/drone_images"
        self.image_name = (f'image{self.i}.png')

        # Adding image to folder
        with open(os.path.join(self.folder_dir, self.image_name), 'wb') as f:
            f.write(msg.data)

        self.publisher.publish(msg)
        self.get_logger().info('Image uploaded')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    drone_image = DroneImage()

    rclpy.spin(drone_image)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
