"""Pedro Ramos"""


def position_mov(x_dist, y_dist) -> list:
    """"Asks user to move drone farther in x/y direction"""
    position = input("Move farther x or y direction (x/y): ")

    if position == "x":
        x_dist = x_dist - 2

    elif position == "y":
        y_dist = y_dist - 2

    return x_dist, y_dist


def drone_points(x_dist: int, y_dist: int, z_dist: int, num_img: int) -> list:
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

    for h in range(1):
        # h == 1 refers to it coming up
        if h == 1:
            x_dist.reverse()
            y_dist.reverse()
            z_dist.reverse()
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
