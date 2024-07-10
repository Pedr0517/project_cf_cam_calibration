import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class VisualPublisher(Node):

    def __init__(self):
        super().__init__("visual_publisher")
        self.publisher = self.create_publisher(
            Marker, "/test_2", 10)
        timer = 1
        self.timer = self.create_timer(timer, self.visual_callback)

    def visual_callback(self):
        msg = Marker()
        msg.header.frame_id = "earth"
        msg.ns = "sphere_shape"
        msg.id = 1
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD
        msg.pose.position.x = 5.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.0
        msg.scale.x = 3.0
        msg.scale.y = 3.0
        msg.scale.z = 3.0
        msg.color.a = 1.0
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Publishing marker: x={msg.pose.position.x}, y={msg.pose.position.y},z={msg.pose.position.z} ")


def main(args=None):
    rclpy.init(args=args)
    visual_publisher = VisualPublisher()

    rclpy.spin(visual_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
