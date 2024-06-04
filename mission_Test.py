#!/bin/python3

"""
mission.py
"""

from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface
from Drone_Path_demo import points
import numpy as np




def drone_run(drone_interface: DroneInterface):
    """ Run the mission """

    speed = 1.0
    takeoff_height = 1.0
    height = 1.0
    data=points
    angle_rad = (np.pi)/2

    sleep_time = 2.0

    dim = 1.0
    path = data

    print("Start mission")

    ##### ARM OFFBOARD #####
    print("Arm")
    drone_interface.offboard()
    sleep(sleep_time)
    print("Offboard")
    drone_interface.arm()
    sleep(sleep_time)

    ##### TAKE OFF #####
    print("Take Off")
    drone_interface.takeoff(takeoff_height, speed=1.0)
    print("Take Off done")
    sleep(sleep_time)

    ##### GO TO #####
    for goal in path:
        print(f"Go to with path facing {goal}")
        drone_interface.go_to.go_to_point_with_yaw(goal, speed=speed,angle = angle_rad)
        sleep(sleep_time)
        print("Go to done")
    sleep(sleep_time)

    ##### LAND #####
    print("Landing")
    drone_interface.land(speed=0.5)
    print("Land done")

    drone_interface.disarm()


if __name__ == '__main__':
    rclpy.init()

    uav = DroneInterface("drone0", verbose=False, use_sim_time=True)

    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
