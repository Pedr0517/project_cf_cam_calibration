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
            CameraInfo, "/drone0/sensor_measurements/camera/camera_info", 10)
        timer = 1
        self.timer = self.create_timer(timer, self.image_callback)

    def image_callback(self):
        msg = CameraInfo()
        msg.header.frame_id = 'drone0/camera/camera_link'
        msg.height = 720
        msg.width = 1280
        msg.distortion_model = "plumb_bob"

        msg.d = [-0.12423564406786976, -0.0019431068572692903, -0.017638379910119486,
                 0.03202173470884484, -0.000933797034617021]

        msg.k = [949.0586006637827, 0.0, 390.93808171241955,
                 0.0, 949.0586006637827, 555.8702679743881,
                 0.0, 0.0, 1.0]

        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]

        msg.p = [949.0586006637827, 0.0, 390.93808171241955, 0.0,
                 0.0, 949.0586006637827, 555.8702679743881, 0.0,
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
