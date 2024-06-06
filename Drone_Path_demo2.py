
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d


#approach#

#create board to stay in the middle
#have points in all three x,y and z
#this will allow us to create a path along those points
#move plot the points so it can create a path which will show the path of the drone


##List of points##
ht_dif = 2
vtcl_ht = 6
horiz_dist = 6
horiz_dif = 2

points = []
x_point = []
y_point = []
z_point = []

##Note, functions are to be used in any orientation, might have to adjust plus or minus in some lines



def drone_down(vtcl_ht,horiz_dist,num_img,dist_away):
    ##ht_dif, refers to height_difference when drone moves through path
    ##vtcl_ht, refers to the height the drone will start at
    ##horiz_dist, refers to the horizontal distance the drone will cover through each session
    ##num_photos, refers to the amount of photos that will be taken at along the segment
    
    ##List##
    x_point = []
    y_point = []
    z_point = []
    x_dist = np.linspace(horiz_dist,2,num_img)
    z_dist = np.linspace(vtcl_ht,0.5,3)
    
    
    
    for i in range(3):
        if 

              


vtcl_ht = 7
horiz_dist = 9
num_img = 4
dist_away = float(input('Enter distance away from charruco board '))
        


horiz_dist = x_data[0]#this allows us to know where the drone left off after finish its path
vtcl_ht = z_data[0]#this allows us to know where the drone left off after finish its path
num_img = 4
dist_away = 6

x_data_2,y_data_2,z_data_2 = drone_up(vtcl_ht,horiz_dist,num_img,dist_away)#we are calling our function again, we do implement array slicing to obtain location of drone

#we are putting data together to obtain plot/sim of drone path
x_plot = x_data + x_data_2
y_plot = y_data + y_data_2
z_plot = z_data + z_data_2


# points=[]#list of points in x,y and z coordinates
# for i in range(len(x_plot)):
#     point = [x_plot[i],y_plot[i],z_plot[i]]
#     points.append(point)
    

    
ax = plt.axes(projection = "3d")
#fig = plt.figure(figsize = (10, 7))
ax.plot3D(x_plot,y_plot,z_plot, color = "green")
ax.scatter(x_plot,y_plot,z_plot, color = "black")





# #this is the size of board
# #note, this is only for simulation purposes

board_x = [7,1,1,7,7]
board_y = [10,10,10,10,10]
board_z = [7,7,0,0,7]

ax.plot3D(board_x,board_y,board_z, color = "blue")
ax.scatter(board_x,board_y,board_z, color = "red")

plt.show()

print(points)



#create fucntions that return a list of points when called
#dont have seperate functions
#change code slightly to meet requirements