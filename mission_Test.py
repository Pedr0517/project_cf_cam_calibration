#!/bin/python3

"""
mission.py
"""

from time import sleep
import argparse
import numpy as np
import rclpy
from as2_python_api.drone_interface import DroneInterface
from drone_functions import get_points, path_plot


def drone_takeoff(drone_interface: DroneInterface):
    """ Run the mission """

    speed = 1.0
    takeoff_height = 1.0

    sleep_time = 2.0

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
    drone_interface.takeoff(takeoff_height, speed=speed)
    print("Take Off done")
    sleep(1)

    # GO TO #


def drone_path(drone_interface: DroneInterface, path_data: list):

    sleep_time = 2.0
    speed = 1.0
    angle_rad = np.pi / 2
    path = path_data

    for goal in path:
        print(f"Go to path {goal}")
        drone_interface.go_to.go_to_point_with_yaw(goal, speed=speed, angle=angle_rad)
        print("Take photo")
        sleep(sleep_time)
        print("Go to done")
    sleep(sleep_time)


def drone_land(drone_interface: DroneInterface):

    # LAND #
    print("Landing")
    drone_interface.land(speed=0.5)
    print("Land done")

    drone_interface.disarm()


if __name__ == '__main__':
    rclpy.init()

    uav = DroneInterface("drone0", verbose=False, use_sim_time=True)

    drone_takeoff(uav)

    # Arguments#
    # For any bounds, use m
    while True:
        # allows for user to have multiple paths

        parser = argparse.ArgumentParser(description='Obtain bounds for drone')

        parser.add_argument('x_max', type=int, help='Max x bound')
        parser.add_argument('x_min', type=int, help='Min x bound')

        parser.add_argument('y_max', type=int, help='Max y bound')
        parser.add_argument('y_min', type=int, help='Min y bound')

        parser.add_argument('z_max', type=int, help='Max z bound')
        parser.add_argument('z_min', type=int, help='Min z bound')

        parser.add_argument('num_img', type=int, help='Number of images')

        args = parser.parse_args()

        # XYZ Bounds#
        x_dist = np.linspace(args.x_max, args.x_min, args.num_img)
        y_dist = np.linspace(args.y_max, args.y_min, args.num_img)
        z_dist = np.linspace(args.z_max, args.z_min, 3)

        num_img = args.num_img

        # If user input origin, loop breaks
        if max(x_dist) == 0 and max(y_dist) == 0 and max(z_dist) == 0:
            break

        # Plot#
        path_plot(x_dist, y_dist, z_dist, num_img)

        data = get_points(x_dist, y_dist, z_dist, num_img)

        drone_path(uav, data)

    drone_land(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
