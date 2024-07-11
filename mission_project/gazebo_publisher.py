import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo


class ImagePublisher(Node):

    def __init__(self):
        super().__init__("image_publisher")
        self.param_use_sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([self.param_use_sim_time])
        self.publisher = self.create_publisher(
            CameraInfo, "/drone0/sensor_measurements/front_camera/camera_info", 10)
        timer = 1
        self.timer = self.create_timer(timer, self.image_callback)

    def image_callback(self):
        msg = CameraInfo()
        msg.header.frame_id = 'drone0/camera/camera_link'
        msg.height = 960
        msg.width = 1280
        msg.distortion_model = "plumb_bob"

        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        msg.k = [1108.5, 0.0, 640.5,
                 0.0, 1108.5, 480.5,
                 0.0, 0.0, 1.0]

        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]

        msg.p = [1108.5, 0.0, 640.5, 0.0,
                 0.0, 1108.5, 480.5, 0.0,
                 0.0, 0.0, 1.0, 0.0]

        self.publisher.publish(msg)
        self.get_logger().info("Publishing info")


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()

    rclpy.spin(image_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
