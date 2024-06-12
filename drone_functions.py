# Drone Functions#
import matplotlib.pyplot as plt
import numpy as np
from drone_path import drone_points
import argparse


def get_points(x_dist: list, y_dist: list, z_dist: list, num_img: int) -> list:
    """"Gathers points from imported functions, returns a list of list containing x,y,z path points"""

    # x_path, collection of all x points within drones path
    # y_path, collection of all y points within drones path
    # z_path, collection of all z points within drones path
    # points, list containing list of points in x,y,z dimensions

    x_path = []
    y_path = []
    z_path = []

    x_path, y_path, z_path = drone_points(x_dist, y_dist, z_dist, num_img)

    # Path Data#

    points = []
    for i in range(len(x_path)):
        point = [x_path[i], y_path[i], z_path[i]]
        points.append(point)

    return points


def path_plot(x_dist: list, y_dist: list, z_dist: list, num_img: int) -> plt:
    """"Simulate path the drone will take using 3d plotting"""

    # x_path, collection of all x points within drones path
    # y_path, collection of all y points within drones path
    # z_path, collection of all z points within drones path

    x_path = []
    y_path = []
    z_path = []

    ax = plt.axes(projection="3d")

    data = get_points(x_dist, y_dist, z_dist, num_img)

    # Path 3D#
    for i in range(len(data)):
        x_path.append(data[i][0])
        y_path.append(data[i][1])
        z_path.append(data[i][2])

    ax.plot3D(x_path, y_path, z_path, color="green")
    ax.scatter(x_path, y_path, z_path, color="black")

    # Charucco Board#
    board_x = [5.4, 6.6, 6.6, 5.4, 5.4]  # [min,min,max,max,min]
    board_y = [9, 9, 9, 9, 9]  # [min,min,max,max,min]
    board_z = [1.2, 1.2, 0, 0, 1.2]

    ax.plot3D(board_x, board_y, board_z, color="blue")
    ax.scatter(board_x, board_y, board_z, color="red")

    # Set Labels#
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_title('3D Drone Path')

    return plt.show()


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Obtain bounds for drone')

    parser.add_argument('--x_max', type=float, default=0.75, help='Max x bound')
    parser.add_argument('--x_min', type=float, default=0.75, help='Min x bound')

    parser.add_argument('--y_max', type=float, default=-0.25, help='Max y bound')
    parser.add_argument('--y_min', type=float, default=0.25, help='Min y bound')

    parser.add_argument('--z_center', type=float, default=0.5, help='Distance away from center')

    parser.add_argument('--num_seg', type=float, default=3, help='Number of segments')

    parser.add_argument('--num_img', type=float, default=3, help='Number of images')

    args = parser.parse_args()

    center_board = [0.6, 9, 0.6]  # [x,y,z]

    # XYZ Bounds#
    x_dist = np.linspace(center_board[0] + args.x_max,
                         center_board[0] - args.x_min, args.num_img)
    y_dist = np.linspace(center_board[1] + args.y_max,
                         center_board[1] - args.y_min, args.num_img)
    z_dist = np.linspace(center_board[2] + args.z_center,
                         center_board[2] - args.z_center, args.num_seg)

    num_img = args.num_img

    # plot_data = get_points(x_dist, y_dist, z_dist, num_img)
    # print(plot_data)
    path_plot(x_dist, y_dist, z_dist, num_img)
