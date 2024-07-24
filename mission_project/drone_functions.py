# Drone Functions#
import argparse
import matplotlib.pyplot as plt
import numpy as np
from drone_path import drone_points


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

    ax.plot3D(x_path, y_path, z_path, color="green", label="Drone Path")
    ax.scatter(x_path, y_path, z_path, color="black", label="Image Captured")

    # Charucco Board#
    board_x = [-0.6, -0.6, 0.6, 0.6, -0.6]
    board_y = [0, 0, 0, 0, 0]
    board_z = [0, 1.2, 1.2, 0, 0]

    ax.plot3D(board_x, board_y, board_z, color="blue", label="Charuco Board")
    ax.scatter(board_x, board_y, board_z, color="red")

    # Set Labels#
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')

    ax.legend(loc="upper left", bbox_to_anchor=(-0.3, 1.1))
    ax.set_title('3D Drone Path')

    return plt.show()


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Obtain bounds for drone')

    parser.add_argument('--x_max', type=float, default=0.60, help='Max x bound')
    parser.add_argument('--x_min', type=float, default=0.60, help='Min x bound')

    parser.add_argument('--y_dist', type=float, default=1.0, help='y bound')

    parser.add_argument('--z_max', type=float, default=0.45, help='Max z bound')
    parser.add_argument('--z_min', type=float, default=0.45, help='Min z bound')

    parser.add_argument('--num_seg', type=int, default=3, help='Number of segments')

    parser.add_argument('--num_img', type=int, default=3, help='Number of images')

    parser.add_argument('--yaw_angle', type=float, default=1.57,
                        help='Yaw drone will take path with')

    args = parser.parse_args()

    center_charruco = [0, 0, 0.6]  # [x,y,z]

    # XYZ Bounds#
    x_dist = np.linspace(center_charruco[0] + args.x_max,
                         center_charruco[0] - args.x_min, args.num_img)
    y_dist = np.linspace(center_charruco[1] - args.y_dist,
                         center_charruco[1] - args.y_dist, args.num_img)
    z_dist = np.linspace(center_charruco[2] + args.z_max,
                         center_charruco[2] - args.z_min, args.num_seg)

    num_img = args.num_img

    # plot_data = get_points(x_dist, y_dist, z_dist, num_img)
    # print(plot_data)
    path_plot(x_dist, y_dist, z_dist, num_img)
