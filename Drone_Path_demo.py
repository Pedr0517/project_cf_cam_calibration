
import numpy as np


# approach#

"""create board to stay in the middle
have points in all three x,y and z
this will allow us to create a path along those points
move plot the points so it can create a path which will
show the path of the drone
"""


def x_path(x_max: int, x_min: int, num_img: int) -> list:
    """"Create list of x points containing those boundaries"""

    return np.linspace(x_max, x_min, num_img)


def y_path(y_max: int, y_min: int, num_img: int) -> list:
    """"Create list of y points containing those boundaries"""

    return np.linspace(y_max, y_min, num_img)


def z_path(z_max: int, z_min: int) -> list:
    """"Create list of z points containing those boundaries"""

    return np.linspace(z_max, z_min, 3)


def position_mov(x_dist, y_dist) -> list:
    """"Asks user to move drone farther in x/y direction"""
    position = input("Move farther x or y direction (x/y): ")

    if position == "x":
        x_dist = x_dist - 2

    elif position == "y":
        y_dist = y_dist - 2

    return x_dist, y_dist


def drone_down(x_dist: int, y_dist: int, z_dist: int, num_img: int) -> list:
    """"Postions drone at top of path, moves it down through x,y,z points"""

    # x_dist, bounds along x axis
    # y_dist, bounds along y axis
    # z_dist, bounds along z axis
    # num_img, photos taken along path at each segment

    # x_point, collection of x points along drones path
    # y_point, collection of y points along drones path
    # z_point, collection of z points along drones path

    # List #
    x_point = []
    y_point = []
    z_point = []

    x_axis = x_dist
    z_axis = z_dist
    y_axis = y_dist

    for i in range(3):
        if i == 0:
            for j in range(num_img):

                x = x_axis[j]
                x_point.append(x)

                z = z_axis[i]
                z_point.append(z)

                y = y_axis[j]
                y_point.append(y)

        if i == 1:
            for j in range(1, num_img + 1):

                x = x_axis[-j]
                x_point.append(x)

                z = z_axis[i]
                z_point.append(z)

                y = y_axis[-j]
                y_point.append(y)

        if i == 2:
            for j in range(num_img):
                x = x_axis[j]
                x_point.append(x)

                z = z_axis[i]
                z_point.append(z)

                y = y_axis[j]
                y_point.append(y)

    return x_point, y_point, z_point


def drone_up(x_dist: int, y_dist: int, z_dist: int, num_img: int) -> list:
    """"Postions drone at bottom of path, moves it up through x,y,z points"""
    # x_dist, bounds along x axis
    # y_dist, bounds along y axis
    # z_dist, bounds along z axis
    # num_img, photos taken along path at each segment

    # x_point_2, collection of x points along drones path
    # y_point_2, collection of y points along drones path
    # z_point_2, collection of z points along drones path

    # List#
    x_point_2 = []
    y_point_2 = []
    z_point_2 = []

    x_axis, y_axis = position_mov(x_dist, y_dist)
    z_axis = z_dist[::-1]

    for i in range(3):
        if i == 0:
            for j in range(1, num_img + 1):
                x = x_axis[-j]
                x_point_2.append(x)

                z = z_axis[i]
                z_point_2.append(z)

                y = y_axis[-j]
                y_point_2.append(y)

        if i == 1:
            for j in range(num_img):

                x = x_axis[j]
                x_point_2.append(x)

                z = z_axis[i]
                z_point_2.append(z)

                y = y_axis[j]
                y_point_2.append(y)

        if i == 2:
            for j in range(1, num_img + 1):
                x = x_axis[-j]
                x_point_2.append(x)

                z = z_axis[i]
                z_point_2.append(z)

                y = y_axis[-j]
                y_point_2.append(y)

    return x_point_2, y_point_2, z_point_2
