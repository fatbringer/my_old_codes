import numpy as np
import cv2
import h5py
f = h5py.File('A1.h5','r')
print(list(f.keys()))

for item in f: 
    print(item) #gets the keys and prints them out 
    continue

#make blank image
height = 768
width = 1024
blank_image = np.zeros((height,width, 3), np.uint8)
blank_image[::] = (255,255,255)
#cv2.imwrite("white.jpg", blank_image)

#extrating data 
attention_data = f.get('attention')
print("attention shape:" , attention_data.shape)
attention_data = np.array(attention_data) # For converting to a NumPy array
print("sum of attention data:", attention_data.sum())
#print(attention_data)



# for (attn_x, attn_y), attention_val in np.ndenumerate(attention_data):
#     #print(attn_x, attn_y, attention_val)
#     continue

density_data = f.get('density')
print("density shape:" , density_data.shape)
density_data = np.array(density_data) # For converting to a NumPy array
print("sum of density data:", density_data.sum())
#print(density_data)



# for (dens_x, dens_y), density_val in np.ndenumerate(density_data):
#     #print(dens_x, dens_y, attention_val)
#     continue
    
attention_max = np.max(attention_data)
density_max = np.max(density_data)

density_times_attention = density_data * attention_data
total = density_times_attention.sum()
print("density_data * attention_data IS", total)

for (x,y), element in np.ndenumerate(density_times_attention):
    maxVal = np.max(density_times_attention)
    
    

gt_data = f.get('gt')
gt_data = np.array(gt_data) # For converting to a NumPy array
print("gt is", gt_data)
print("gt is of type", type(gt_data))


# #saving to text file 
# attention = open("h5attention.csv", 'w')
# np.savetxt(attention, attention_data, delimiter=',' , fmt='%10.5f')
# attention.close()

# density = open("h5density.csv", 'w')
# np.savetxt(density, density_data, delimiter=',', fmt='%10.5f')
# density.close()

# gtFile = open("h5GT.txt", 'w')
# np.savetxt(gtFile, gt_data, fmt='%.2f')
# gtFile.close()

