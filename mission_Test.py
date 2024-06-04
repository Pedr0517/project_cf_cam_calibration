#!/bin/python3

"""
mission.py
"""

from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface
from drone_functions import get_points,path_plot
from Drone_Path_demo import horiz_path,vert_path
import numpy as np




def drone_run(drone_interface: DroneInterface):
    """ Run the mission """

    speed = 1.0
    takeoff_height = 1.0
    height = 1.0

    # vtcl_ht = int(input("Énter verticle height: "))
    # horiz_dist = int(input("Énter horizontal distance covered: "))
    # num_img = int(input("Énter number of images: "))
    # dist_away = int(input("Énter distance away relative to board: "))

    vtcl_ht = 6
    num_img = 4
    horiz_dist = horiz_path(8,3,num_img)#prior to running mission, edit x_max/x_min to desired reference axis
    dist_away = vert_path(4,4,num_img)#prior to running mission, edit y_max/y_min to desired reference axis

    #path_plot(vtcl_ht,horiz_dist,num_img,dist_away)

    data=get_points(vtcl_ht,horiz_dist,num_img,dist_away)
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
    sleep(1)

    ##### GO TO #####
    for goal in path:
        print(f"Go to path {goal}")
        drone_interface.go_to.go_to_point_with_yaw(goal, speed=speed,angle = angle_rad)
        print("Take photo")
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
