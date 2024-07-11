"""Pedro Ramos"""


def drone_points(x_dist: list, y_dist: list, z_dist: list, num_img: int) -> list:
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

    for i in range(len(z_dist)):
        if i % 2 == 0:

            for j in range(num_img):

                x_point.append(x_dist[j])

                z_point.append(z_dist[i])

                y_point.append(y_dist[j])

        if i % 2 != 0:
            for j in range(1, num_img + 1):

                x_point.append(x_dist[-j])

                z_point.append(z_dist[i])

                y_point.append(y_dist[-j])

    return x_point, y_point, z_point
