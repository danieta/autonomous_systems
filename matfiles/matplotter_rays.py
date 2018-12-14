#%%

import numpy as np
from scipy import io as sio
import matplotlib.pyplot as plt
import cv2

data1 = sio.loadmat('hall_straight_51_rays')
data2 = sio.loadmat('hall_straight_good_initial_belief')
data3 = sio.loadmat('hall_straight_9_rays')
data4 = sio.loadmat('hall_straight_3_rays')
data5 = sio.loadmat('hall_straight_301_rays')

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

positions4 = data4['beliefs'][:-1,0:2]/data4['map_resolution']
predictions4 = data4['predictions'][:-1,0:2]/data4['map_resolution']
positions4[:,1] = 550-positions4[:,1]
predictions4[:,1] = 550-predictions4[:,1]

positions5 = data5['beliefs'][:-1,0:2]/data5['map_resolution']
predictions5 = data5['predictions'][:-1,0:2]/data5['map_resolution']
positions5[:,1] = 550-positions5[:,1]
predictions5[:,1] = 550-predictions5[:,1]


plt.figure()

img = cv2.imread('map_with_edits.pgm',0)
plt.imshow(img, cmap = 'gray', interpolation = 'bicubic')
plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis

plt.plot(positions5[:,0], positions5[:,1], 'c,-', linewidth=5, label="Belief 301 rays")

plt.plot(positions1[:,0], positions1[:,1], 'r,-', linewidth=5, label="Belief 51 rays")
#plt.plot(predictions1[:,0], predictions1[:,1], 'r,--', linewidth=5, label="Prediction trajectory 1")



plt.plot(positions3[:,0], positions3[:,1], 'b,-', linewidth=5, label="Belief 9 rays")
#plt.plot(predictions3[:,0], predictions3[:,1], 'b,--', linewidth=5, label="Prediction trajectory 3")

plt.plot(positions4[:,0], positions4[:,1], 'y,-', linewidth=5, label="Belief 3 rays")


plt.plot(positions2[:,0], positions2[:,1], 'g,-', linewidth=5, label="Ground truth")
#plt.plot(predictions2[:,0], predictions2[:,1], 'g,--', linewidth=5, label="Prediction trajectory 2")

plt.legend()
plt.show()