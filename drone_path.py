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

    for i in range(3):
        if i == 0 or i == 2:

            for j in range(num_img):
                x = x_dist[j]
                x_point.append(x)

                z = z_dist[i]
                z_point.append(z)

                y = y_dist[j]
                y_point.append(y)

        if i == 1:
            for j in range(1, num_img + 1):

                x = x_dist[-j]
                x_point.append(x)

                z = z_dist[i]
                z_point.append(z)

                y = y_dist[-j]
                y_point.append(y)

    return x_point, y_point, z_point
