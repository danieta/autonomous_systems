#%%

import numpy as np
from scipy import io as sio
import matplotlib.pyplot as plt
import cv2

data1 = sio.loadmat('ground_truth1')
data2 = sio.loadmat('ground_truth2')
data3 = sio.loadmat('ground_truth3')

origin = data1['map_origin'][:,0:2]
delta_truth = data1['ground_truth'][0,:]

print origin

positions1 = (data3['beliefs'][:-1,0:2]-origin)/data1['map_resolution']
predictions1 = (data3['predictions'][:-1,0:2]-origin)/data1['map_resolution']
ground_truth1 = data3['ground_truth']-delta_truth
ground_truth1 = (ground_truth1[:-1,0:2]-origin)/data1['map_resolution']
positions1[:,1] = 1056-positions1[:,1]
predictions1[:,1] = 1056-predictions1[:,1]
ground_truth1[:,1] = 1056-ground_truth1[:,1]

# positions2 = data2['beliefs'][:-1,0:2]/data2['map_resolution']
# predictions2 = data2['predictions'][:-1,0:2]/data2['map_resolution']
# positions2[:,1] = 550-positions2[:,1]
# predictions2[:,1] = 550-predictions2[:,1]

# positions3 = data3['beliefs'][:-1,0:2]/data3['map_resolution']
# predictions3 = data3['predictions'][:-1,0:2]/data3['map_resolution']
# positions3[:,1] = 550-positions3[:,1]
# predictions3[:,1] = 550-predictions3[:,1]

plt.figure()

img = cv2.imread('8floor_edited.pgm',0)
plt.imshow(img, cmap = 'gray', interpolation = 'bicubic')
plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis

#plt.scatter(positions1[:,0], positions1[:,1], c=data1['matches'][:-1], linewidth=5)
plt.plot(positions1[:,0], positions1[:,1], 'r,-', linewidth=2, label="Belief trajectory")
#plt.scatter(predictions1[:,0], predictions1[:,1], c=data1['matches'][:-1], linewidth=5)
plt.plot(predictions1[:,0], predictions1[:,1], 'r,--', linewidth=2, label="Prediction trajectory")

plt.plot(ground_truth1[:,0], ground_truth1[:,1], 'b,-', linewidth=2, label="Ground truth")

#plt.scatter(positions2[:,0], positions2[:,1], c=data2['matches'][:-1], linewidth=5)
#plt.plot(positions2[:,0], positions2[:,1], 'g,-', linewidth=5, label="Belief trajectory 2")
#plt.scatter(predictions2[:,0], predictions2[:,1], c= data2['matches'][:-1], linewidth=5)
#plt.plot(predictions2[:,0], predictions2[:,1], 'g,--', linewidth=5, label="Prediction trajectory 2")

#plt.scatter(positions3[:,0], positions3[:,1], c=data3['matches'][:-1], linewidth=5)
#plt.plot(positions3[:,0], positions3[:,1], 'b,-', linewidth=5, label="Belief trajectory 3")
#plt.scatter(predictions3[:,0], predictions3[:,1], c=data3['matches'][:-1], linewidth=5)
#plt.plot(predictions3[:,0], predictions3[:,1], 'b,--', linewidth=5, label="Prediction trajectory 3")


plt.legend()
plt.show()