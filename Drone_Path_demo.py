
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d


#approach#

#create board to stay in the middle
#have points in all three x,y and z
#this will allow us to create a path along those points
#move plot the points so it can create a path which will show the path of the drone







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
        if i == 0:
            for j in range(num_img):
               
                x = x_dist[j]
                x_point.append(x)
                
                z = z_dist[i]
                z_point.append(z)
                
                y = dist_away
                y_point.append(y)
                
                #horiz_dist -= (x_dist/num_photos)# we subtract becuse we are moving right to left
                
        if i == 1:
            for j in range(1,num_img+1):
                #horiz_dist += (x_dist/num_photos)#we add becasue we are moving left to right
                x = x_dist[-j]
                x_point.append(x)
                
                z = z_dist[i]#this allows drone to move down/up, in this instance, we are moving down therefore we subtract
                z_point.append(z)
                
                y = dist_away
                y_point.append(y)
    
        if i == 2:
              for j in range(num_img):
                  x = x_dist[j]
                  x_point.append(x)
                  
                  z = z_dist[i]
                  z_point.append(z)
                  
                  y = dist_away
                  y_point.append(y)
                  
                 # horiz_dist -= (x_dist/num_photos)
    return x_point,y_point,z_point

def drone_up(vtcl_ht,horiz_dist,num_img,dist_away):
    
    dist_away = dist_away - 4##note, this is used in order to obtian images at a farther distance
    
    ##List##
    x_point_2 = []
    y_point_2 = []
    z_point_2 = []
    x_dist = np.linspace(horiz_dist,2,num_img)
    z_dist = np.linspace(0.5,vtcl_ht,3)

    
    for i in range(3):
        if i == 0:
            for j in range(1,num_img+1):
                x = x_dist[-j]
                x_point_2.append(x)
                
                z = z_dist[i]
                z_point_2.append(z)
                
                y = dist_away
                y_point_2.append(y)
                
               # horiz_dist += (x_dist/num_photos)
                
                
        if i == 1:
            for j in range(num_img):
               # horiz_dist -= (x_dist/num_photos)
                x = x_dist[j]
                x_point_2.append(x)
                
                z = z_dist[i]
                z_point_2.append(z)
                
                y = dist_away
                y_point_2.append(y)
    
        if i == 2:
              for j in range(1,num_img+1):
                  x = x_dist[-j]
                  x_point_2.append(x)
                  
                  z = z_dist[i]
                  z_point_2.append(z)
                  
                  y = dist_away
                  y_point_2.append(y)
                  
                  #horiz_dist += (x_dist/num_photos)
    return x_point_2,y_point_2,z_point_2
              

vtcl_ht = 7
horiz_dist = 8
num_img = 4
dist_away = 6
            
        
x_data,y_data,z_data = drone_down(vtcl_ht,horiz_dist,num_img,dist_away)#we are calling our function with known values

x_data_2,y_data_2,z_data_2 = drone_up(vtcl_ht,horiz_dist,num_img,dist_away)#we are calling our function again, we do implement array slicing to obtain location of drone

#we are putting data together to obtain plot/sim of drone path
x_plot = x_data + x_data_2
y_plot = y_data + y_data_2
z_plot = z_data + z_data_2


points=[]#list of points in x,y and z coordinates
for i in range(len(x_plot)):
    point = [x_plot[i],y_plot[i],z_plot[i]]
    points.append(point)
    

    
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

# print(points)



#create fucntions that return a list of points when called
#dont have seperate functions
#change code slightly to meet requirements