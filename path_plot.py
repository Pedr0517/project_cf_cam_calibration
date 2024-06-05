# import matplotlib.pyplot as plt
# import numpy as np


# x_dist = [1, 1, 1, 1, 1,]
# y_dist = [1, 1, 1, 1, 1,]
# z_dist = [1, 1, 1, 1, 1,]


# ax = plt.axes(projection="3d")

# board_x = [max(x_dist), 0, 0, max(x_dist), max(x_dist)]
# board_y = np.linspace(max(y_dist), min(y_dist), 5)
# board_z = [max(z_dist), max(z_dist), 0, 0, max(z_dist)]

# ax.plot3D(board_x, board_y, board_z, color="blue")
# ax.scatter(board_x, board_y, board_z, color="red")

# plt.show()

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the dimensions of the square
side_length = 1
height = 2

# Plot the square
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Define the vertices of the square
vertices = np.array([
    [0, 0, 0],
    [side_length, 0, 0],
    [side_length, side_length, 0],
    [0, side_length, 0],
    [0, 0, height],
    [side_length, 0, height],
    [side_length, side_length, height],
    [0, side_length, height],
    [0, 0, 0]  # To close the square
])

# Plot the vertices
ax.plot(vertices[:, 0], vertices[:, 1], vertices[:, 2], color='blue')

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
