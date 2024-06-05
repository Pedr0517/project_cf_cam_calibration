#!/bin/python3

"""
mission.py
"""

from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface
from drone_functions import get_points, path_plot
from Drone_Path_demo import x_path, y_path, z_path
import numpy as np
import argparse


def drone_run(drone_interface: DroneInterface, data: list):
    """ Run the mission """

    speed = 1.0
    takeoff_height = 1.0

    data = data
    yaw = [(np.pi) / 2, (np.pi * 3) / 4, np.pi / 4]

    sleep_time = 2.0

    path = data

    print("Start mission")

    # ARM OFFBOARD #
    print("Arm")
    drone_interface.offboard()
    sleep(sleep_time)
    print("Offboard")
    drone_interface.arm()
    sleep(sleep_time)

    # TAKE OFF #
    print("Take Off")
    drone_interface.takeoff(takeoff_height, speed=1.0)
    print("Take Off done")
    sleep(1)

    # GO TO #

    for angle_rad in yaw:
        for goal in path:
            print(f"Go to path {goal}")
            drone_interface.go_to.go_to_point_with_yaw(goal, speed=speed, angle=angle_rad)
            print("Take photo")
            sleep(sleep_time)
            print("Go to done")
        sleep(sleep_time)

    # LAND #
    print("Landing")
    drone_interface.land(speed=0.5)
    print("Land done")

    drone_interface.disarm()


if __name__ == '__main__':
    rclpy.init()
    """For any bounds, use cm"""

    parser = argparse.ArgumentParser(description='Obtain bounds for drone')

    parser.add_argument('x_max', type=int, help='Max x bound')
    parser.add_argument('x_min', type=int, help='Min x bound')

    parser.add_argument('y_max', type=int, help='Max y bound')
    parser.add_argument('y_min', type=int, help='Min y bound')

    parser.add_argument('z_max', type=int, help='Max z bound')
    parser.add_argument('z_min', type=int, help='Min z bound')

    parser.add_argument('num_img', type=int, help='Number of images')

    args = parser.parse_args()

    x_dist = x_path(args.x_max, args.x_min, args.num_img)
    y_dist = y_path(args.y_max, args.y_min, args.num_img)
    z_dist = z_path(args.z_max, args.z_min, 3)
    num_img = args.num_img

    path_plot(x_dist, y_dist, z_dist, num_img)

    data = get_points(x_dist, y_dist, z_dist, num_img)

    uav = DroneInterface("drone0", verbose=False, use_sim_time=True)

    drone_run(uav, data)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
