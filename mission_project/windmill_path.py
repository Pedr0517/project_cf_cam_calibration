import rclpy
from as2_python_api.drone_interface import DroneInterface
from time import sleep
import argparse
from sensor_msgs.msg import Image
import numpy as np
import os
import cv2


class DroneInspector(DroneInterface):
    def __init__(self, drone_id: str = "drone0", verbose: bool = False, use_sim_time: bool = False):
        super().__init__(drone_id=drone_id, verbose=verbose, use_sim_time=use_sim_time)

        self.create_subscription(
            Image, '/drone0/sensor_measurements/front_camera/image_raw', self.image_upload, 10)

        self.i = 0

    def image_upload(self, msg: Image):

        self.image_received = msg

    def save_image(self):
        self.get_logger().info('Image received')

        # Location and name
        folder_dir = "windmill_images"

        # Converting ROS images to compatible file
        image_np = np.frombuffer(self.image_received.data, dtype=np.uint8)

        image_np = image_np.reshape((self.image_received.height,
                                     self.image_received.width, -1))

        # Location and name
        image_name = f'image_taken{self.i}.png'
        image_path = os.path.join(folder_dir, image_name)

        # Checking directory
        if not os.path.exists(folder_dir):
            os.makedirs(folder_dir)

        # Adding image to folder
        cv2.imwrite(image_path, image_np)

        self.get_logger().info('Image uploaded')
        self.i += 1


def windmill_mission(drone_inspector: DroneInspector, path_data: list, angle: float):

    speed = 0.5
    takeoff_height = 1.0
    path = path_data
    angle = angle

    sleep_time = 2.0

    print("Start Inspection")

    print("Arm")
    drone_inspector.offboard()
    sleep(sleep_time)
    print("Offboard")
    drone_inspector.arm()
    sleep(sleep_time)

    # TAKE OFF #
    print("Take Off")
    drone_inspector.takeoff(takeoff_height, speed=speed)
    print("Take Off done")
    sleep(1)

    # INSPECTION PATH#

    for goal in path[0][1]:
        print(f"Go to point {list(goal)}")
        drone_inspector.go_to.go_to_point_with_yaw(list(goal), speed=speed, angle=angle)
        sleep(sleep_time)
        drone_inspector.save_image()
        sleep(sleep_time)
    # LANDING#
    print("Landing")
    drone_inspector.land(speed=0.5)
    print("Land done")


if __name__ == '__main__':
    rclpy.init()

    path = []
    path.append((0, [(-2, 0, 1), (-2, 0, 3), (-2, 0, 5)]))

    print(path[0][1])

    # path.append(0, (-1, 0, 1))
    # path.append(0, (-1, 0, 3))
    # path.append(0, (-1, 0, 5))

    uav = DroneInterface("drone0", verbose=False, use_sim_time=True)

    parser = argparse.ArgumentParser(description='Yaw of Drone')
    parser.add_argument('--yaw_angle', type=float, default=0, help='Yaw for drone')

    args = parser.parse_args()

    windmill_mission(uav, path, args.yaw_angle)

    uav.shutdown()
    rclpy.shutdown()
    print("clean exit")
