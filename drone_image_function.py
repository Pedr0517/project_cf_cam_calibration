import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class DroneImage(Node):
    def __init__(self):
        super().__init__('drone_image')
        self.publisher = self.create_publisher(Image, 'topic', 1)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.images = []

    def timer_callback(self):
        msg = Image()
        msg.data = 'image here'
        self.publisher.publish(msg)
        self.get_logger().info('Image uploaded')
        self.images.append(msg.data)


def main(args=None):
    rclpy.init(args=args)

    drone_image = DroneImage()

    rclpy.spin(drone_image)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
