
from Drone_Path_demo import drone_down
from Drone_Path_demo import drone_up
import matplotlib.pyplot as plt
import numpy as np

from Drone_Path_demo import horiz_path,vert_path






def get_points(vtcl_ht,horiz_dist,num_img,dist_away):

    x_path = []
    y_path = []
    z_path = []


    x_data,y_data,z_data = drone_down(vtcl_ht,horiz_dist,num_img,dist_away)

    x_data_2,y_data_2,z_data_2 = drone_up(vtcl_ht,horiz_dist,num_img,dist_away)

    x_path = x_data + x_data_2
    y_path = y_data + y_data_2
    z_path = z_data + z_data_2

    points = []#list of points in x,y and z coordinates
    for i in range(len(x_path)):
        point = [x_path[i],y_path[i],z_path[i]]
        points.append(point)

    return points



def path_plot(vtcl_ht,horiz_dist,num_img,dist_away):

    x_path = []
    y_path = []
    z_path = []

    ax = plt.axes(projection = "3d")
 
    data = get_points(vtcl_ht,horiz_dist,num_img,dist_away)

    for i in range(len(data)):
        x_path.append(data[i][0])
        y_path.append(data[i][1])
        z_path.append(data[i][2])
    

    
    ax.plot3D(x_path,y_path,z_path, color = "green")
    ax.scatter(x_path,y_path,z_path, color = "black")

    board_x = [max(horiz_dist),0,0,max(horiz_dist),max(horiz_dist)]
    board_y = np.linspace(max(dist_away)+2,min(dist_away)+2,5)
    board_z = [max(horiz_dist),max(horiz_dist),0,0,max(horiz_dist)]

    ax.plot3D(board_x,board_y,board_z, color = "blue")
    ax.scatter(board_x,board_y,board_z, color = "red")

    

    return plt.show()



