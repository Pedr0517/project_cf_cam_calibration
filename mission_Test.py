#!/bin/python3

"""
mission.py
"""

from time import sleep
import argparse
import os
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from as2_python_api.drone_interface import DroneInterface
import cv2
from sensor_msgs.msg import Image
from drone_functions import get_points, path_plot


class DroneInspector(DroneInterface):
    def __init__(self, drone_id: str = "drone0", verbose: bool = False, use_sim_time: bool = False):
        super().__init__(drone_id=drone_id, verbose=verbose, use_sim_time=use_sim_time)

        self.box_position = None
        self.create_subscription(PoseStamped, '/box_0/box_0/pose', self.box_pose_callback, 10)
        self.create_subscription(
            Image, 'sensor_measurements/cam/image_raw', self.image_upload, 10)
        print("drone_inspector intitialized")

        self.i = 0

    def box_pose_callback(self, msg: PoseStamped):
        self.box_position = msg.pose.position
        # print(f"Box position: {self.box_position}")  # Debugging

    def image_upload(self, msg):

        self.image_received = msg

    def save_image(self):
        self.get_logger().info('Image received')

        # Location and name
        folder_dir = "drone_images"

        # Converting ROS images to compatible file
        image_np = np.frombuffer(self.image_received.data, dtype=np.uint8)

        image_np = image_np.reshape((self.image_received.height, self.image_received.width, -1))

        # Location and name
        image_name = f'image_taken{self.i}.png'
        image_path = os.path.join(folder_dir, image_name)

        # Adding image to folder
        cv2.imwrite(image_path, image_np)

        self.get_logger().info('Image uploaded')
        self.i += 1


def drone_path(drone_inspector: DroneInspector, path_data: list, angle: float):
    """ Run the mission """
    # [0.0:0, np.pi/2:1.57, np.pi:3.14, (np.pi*3)/2:4.71]
    speed = 0.5
    takeoff_height = 1.0
    path = path_data
    angle_rad = angle

    sleep_time = 2.0

    print("Start mission")

    # ARM OFFBOARD #
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

    # PATH #
    while True:

        for goal in path:
            print(f"Go to path {goal}")
            drone_inspector.go_to.go_to_point_with_yaw(goal, speed=speed, angle=angle_rad)
            sleep(sleep_time)
            drone_inspector.save_image()
            print("Go to done")
        sleep(sleep_time)

        if input('Repeat path (Y/n)? ') == 'n':
            break

    # LAND #
    print("Go to origin")
    drone_inspector.go_to.go_to_path_facing(0.0, 0.0, 1.0, speed=speed)  # ask about this
    sleep(sleep_time)
    print("Landing")
    drone_inspector.land(speed=0.5)
    print("Land done")

    drone_inspector.disarm()


if __name__ == '__main__':
    rclpy.init()

    uav = DroneInspector("drone0", verbose=False, use_sim_time=True)
    sleep(3)
    # Arguments#
    # For any bounds, use m
    # allows for user to have multiple paths

    parser = argparse.ArgumentParser(description='Obtain bounds for drone')

    parser.add_argument('--x_max', type=int, default=3, help='Max x bound')
    parser.add_argument('--x_min', type=int, default=3, help='Min x bound')

    parser.add_argument('--y_max', type=int, default=1, help='Max y bound')
    parser.add_argument('--y_min', type=int, default=1, help='Min y bound')

    parser.add_argument('--z_center', type=int, default=3, help='Distance away from center')

    parser.add_argument('--num_seg', type=int, default=3, help='Number of segments')

    parser.add_argument('--num_img', type=int, default=3, help='Number of images')

    parser.add_argument('--yaw_angle', type=float, default=0, help='Yaw drone will take path with')

    args = parser.parse_args()

    # XYZ Bounds#

    center_charruco = [uav.box_position.x, uav.box_position.y, uav.box_position.z]

    x_dist = np.linspace(center_charruco[0] + args.x_max,
                         center_charruco[0] - args.x_min, args.num_img)
    y_dist = np.linspace(center_charruco[1] + args.y_max,
                         center_charruco[1] - args.y_min, args.num_img)
    z_dist = np.linspace(center_charruco[2] + args.z_center,
                         center_charruco[2] - args.z_center, args.num_seg)

    num_img = args.num_img

    # PLOT #
    path_plot(x_dist, y_dist, z_dist, num_img)

    # DATA #
    data = get_points(x_dist, y_dist, z_dist, num_img)

    drone_path(uav, data, args.yaw_angle)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
