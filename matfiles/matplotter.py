#%%

import numpy as np
from scipy import io as sio
import matplotlib.pyplot as plt
import cv2

data1 = sio.loadmat('matfile_bag_without_map')
data2 = sio.loadmat('matfile_bag_turn_and_move_straight')
data3 = sio.loadmat('matfile_bag_move_straight')

positions1 = data1['beliefs'][:-1,0:2]/data1['map_resolution']
predictions1 = data1['predictions'][:-1,0:2]/data1['map_resolution']
positions1[:,1] = 550-positions1[:,1]
predictions1[:,1] = 550-predictions1[:,1]

positions2 = data2['beliefs'][:-1,0:2]/data2['map_resolution']
predictions2 = data2['predictions'][:-1,0:2]/data2['map_resolution']
positions2[:,1] = 550-positions2[:,1]
predictions2[:,1] = 550-predictions2[:,1]

positions3 = data3['beliefs'][:-1,0:2]/data3['map_resolution']
predictions3 = data3['predictions'][:-1,0:2]/data3['map_resolution']
positions3[:,1] = 550-positions3[:,1]
predictions3[:,1] = 550-predictions3[:,1]

plt.figure()

img = cv2.imread('map_with_edits.pgm',0)
plt.imshow(img, cmap = 'gray', interpolation = 'bicubic')
plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis

#plt.scatter(positions1[:,0], positions1[:,1], c=data1['matches'][:-1], linewidth=5)
plt.plot(positions1[:,0], positions1[:,1], 'r,-', linewidth=5, label="Belief trajectory 1")
#plt.scatter(predictions1[:,0], predictions1[:,1], c=data1['matches'][:-1], linewidth=5)
plt.plot(predictions1[:,0], predictions1[:,1], 'r,--', linewidth=5, label="Prediction trajectory 1")

#plt.scatter(positions2[:,0], positions2[:,1], c=data2['matches'][:-1], linewidth=5)
plt.plot(positions2[:,0], positions2[:,1], 'g,-', linewidth=5, label="Belief trajectory 2")
#plt.scatter(predictions2[:,0], predictions2[:,1], c= data2['matches'][:-1], linewidth=5)
plt.plot(predictions2[:,0], predictions2[:,1], 'g,--', linewidth=5, label="Prediction trajectory 2")

#plt.scatter(positions3[:,0], positions3[:,1], c=data3['matches'][:-1], linewidth=5)
plt.plot(positions3[:,0], positions3[:,1], 'b,-', linewidth=5, label="Belief trajectory 3")
#plt.scatter(predictions3[:,0], predictions3[:,1], c=data3['matches'][:-1], linewidth=5)
plt.plot(predictions3[:,0], predictions3[:,1], 'b,--', linewidth=5, label="Prediction trajectory 3")


plt.legend()
plt.show()