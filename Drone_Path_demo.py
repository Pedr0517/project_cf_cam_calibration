
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d


#approach#

#create board to stay in the middle
#have points in all three x,y and z
#this will allow us to create a path along those points
#move plot the points so it can create a path which will show the path of the drone

def horiz_path(x_max,x_min,num_img):
    return np.linspace(x_max,x_min,num_img)

def vert_path(y_max,y_min,num_img):
    return np.linspace(y_max,y_min,num_img)

def position_mov(x_dist,y_dist):
    position = input("Move farther/closer x or y direction (x/y): ")

    if position == "x":
        x_dist = x_dist - 2
        
    elif position == "y":
        y_dist = y_dist - 2
        
    return x_dist,y_dist





def drone_down(vtcl_ht,horiz_dist,num_img,dist_away):
    ##ht_dif, refers to height_difference when drone moves through path
    ##vtcl_ht, refers to the height the drone will start at
    ##horiz_dist, refers to the horizontal distance the drone will cover through each session
    ##num_photos, refers to the amount of photos that will be taken at along the segment
    
    ##List##
    x_point = []
    y_point = []
    z_point = []
    
    x_axis = horiz_dist
    z_dist = np.linspace(vtcl_ht,0.5,3)
    y_axis = dist_away
    
    
    for i in range(3):
        if i == 0:
            for j in range(num_img):
               
                x = x_axis[j]
                x_point.append(x)
                
                z = z_dist[i]
                z_point.append(z)
                
                y = y_axis[j]
                y_point.append(y)
                
                #horiz_dist -= (x_dist/num_photos)# we subtract becuse we are moving right to left
                
        if i == 1:
            for j in range(1,num_img+1):
                #horiz_dist += (x_dist/num_photos)#we add becasue we are moving left to right
                x = x_axis[-j]
                x_point.append(x)
                
                z = z_dist[i]#this allows drone to move down/up, in this instance, we are moving down therefore we subtract
                z_point.append(z)
                
                y = y_axis[-j]
                y_point.append(y)
    
        if i == 2:
              for j in range(num_img):
                  x = x_axis[j]
                  x_point.append(x)
                  
                  z = z_dist[i]
                  z_point.append(z)
                  
                  y = y_axis[j]
                  y_point.append(y)
                  
                 # horiz_dist -= (x_dist/num_photos)
    return x_point,y_point,z_point

def drone_up(vtcl_ht,horiz_dist,num_img,dist_away):
    
    ##List##
    x_point_2 = []
    y_point_2 = []
    z_point_2 = []
    
    x_axis,y_axis = position_mov(horiz_dist,dist_away)
    z_dist = np.linspace(0.5,vtcl_ht,3)
    
    

    
    for i in range(3):
        if i == 0:
            for j in range(1,num_img+1):
                x = x_axis[-j]
                x_point_2.append(x)
                
                z = z_dist[i]
                z_point_2.append(z)
                
                y = y_axis[-j]
                y_point_2.append(y)
                
               # horiz_dist += (x_dist/num_photos)
                
                
        if i == 1:
            for j in range(num_img):
               # horiz_dist -= (x_dist/num_photos)
                x = x_axis[j]
                x_point_2.append(x)
                
                z = z_dist[i]
                z_point_2.append(z)
                
                y = y_axis[j]
                y_point_2.append(y)
    
        if i == 2:
              for j in range(1,num_img+1):
                  x = x_axis[-j]
                  x_point_2.append(x)
                  
                  z = z_dist[i]
                  z_point_2.append(z)
                  
                  y = y_axis[-j]
                  y_point_2.append(y)
                  
                  #horiz_dist += (x_dist/num_photos)
    return x_point_2,y_point_2,z_point_2
              



