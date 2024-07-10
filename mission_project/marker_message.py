import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class VisualPublisher(Node):

    def __init__(self):
        super().__init__("visual_publisher")
        self.publisher = self.create_publisher(
            Marker, "/test_1", 10)
        timer = 1
        self.timer = self.create_timer(timer, self.visual_callback)

    def visual_callback(self):
        # MARKER 1#
        msg = Marker()
        msg.header.frame_id = "earth"
        msg.ns = "line_strip_1"
        msg.id = 0
        msg.type = Marker.LINE_STRIP
        msg.action = Marker.ADD
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.scale.x = 0.1
        msg.scale.y = 0.0
        msg.scale.z = 0.0
        msg.color.a = 1.0
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0

        msg.points = []
        first_line_point = Point()
        first_line_point.x = -2.0
        first_line_point.y = 0.0
        first_line_point.z = 5.0
        msg.points.append(first_line_point)

        second_line_point = Point()
        second_line_point.x = 2.0
        second_line_point.y = 0.0
        second_line_point.z = 5.0
        msg.points.append(second_line_point)

        # MARKER 2#
        msg2 = Marker()
        msg2.header.frame_id = "earth"
        msg2.ns = "line_strip_2"
        msg2.id = 1
        msg2.type = Marker.LINE_STRIP
        msg2.action = Marker.ADD
        msg2.pose.position.x = 0.0
        msg2.pose.position.y = 0.0
        msg2.pose.position.z = 0.0
        msg2.pose.orientation.x = 0.0
        msg2.pose.orientation.y = 0.0
        msg2.pose.orientation.z = 0.0
        msg2.pose.orientation.w = 1.0
        msg2.scale.x = 0.1
        msg2.scale.y = 0.0
        msg2.scale.z = 0.0
        msg2.color.a = 1.0
        msg2.color.r = 1.0
        msg2.color.g = 0.0
        msg2.color.b = 0.0

        msg2.points = []
        first_line_point_2 = Point()
        first_line_point_2.x = 0.0
        first_line_point_2.y = -2.0
        first_line_point_2.z = 5.0
        msg.points.append(first_line_point_2)

        second_line_point_2 = Point()
        second_line_point_2.x = 0.0
        second_line_point_2.y = 2.0
        second_line_point_2.z = 5.0
        msg.points.append(second_line_point_2)

        self.publisher.publish(msg)
        self.publisher.publish(msg2)
        # self.get_logger().info(
        #     f"Publishing marker: x={msg.pose.position.x}, y={msg.pose.position.y},z={msg.pose.position.z} ")


def main(args=None):
    rclpy.init(args=args)
    visual_publisher = VisualPublisher()

    rclpy.spin(visual_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
