import scipy
from scipy.io import loadmat
import pandas as pd
import numpy as np
import cv2 

#specify your file directories here 
img_dir = "IMG_50.jpg" 
matfile_dir = 'GT_IMG_50.mat'

#opening base image to draw on later 
input_image = cv2.imread(img_dir)

#loads the .mat file using scipy
matContent = scipy.io.loadmat(matfile_dir) 

#mat file is labelled in a certain array format. 
#array format goes ['image_info'][0][0][0][0][x], where x is any number 
# x = 0 gives the x,y coordinates of the points
# x = 1 gives the ground truth of the crowd count 

coordinates = matContent['image_info'][0][0][0][0][0] #extracts coordinates of heads 
print("coordinates are", coordinates)

for item in coordinates:
    #print(item) #item will be the xy coodinates of each data point
    xpoint = int(item[0])
    ypoint = int(item[1])

    #we want to draw these coordinates as dots on the image for visualisation. 
    radius = 3
    colour = (0,255,0) #BGR 
    thickness = -1
    added_circles  = cv2.circle(input_image , (xpoint,ypoint), radius, colour, thickness) #assigns the base image with drawn circles to another variable

output_filename = img_dir + "GREENDOTS.jpg" 
cv2.imwrite(output_filename, added_circles) #saves the base image with green circles drawn to a different filename 

gt = matContent['image_info'][0][0][0][0][1]  #extracts the Ground Truth
print("gt is", gt) #prints ground truth 
