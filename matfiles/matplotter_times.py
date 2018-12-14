#%%

import numpy as np
from scipy import io as sio
import matplotlib.pyplot as plt
import cv2

data_51_rays = sio.loadmat('hall_straight_51_rays')
data_9_rays = sio.loadmat('hall_straight_9_rays')
data_3_rays = sio.loadmat('hall_straight_3_rays')
data_301_rays = sio.loadmat('hall_straight_301_rays')
data_151_rays = sio.loadmat('hall_straight_151_rays')



def avg_iteration_time(data):

	# AVERAGE TIME BETWEEN ITERATIONS
	average_time_iteration = np.sum(data['times']) / (data['times'].shape[1] - 1) #-1 to remove the 0 in the beginning

	#print("iterations")
	#print(average_time_iteration)
	return average_time_iteration


def avg_matches_time(data):
	# AVERAGE TIME BETWEEN MATCHES
	iteration_time_matches = 0
	number_of_matches = 0
	for i in range(1, data['matches'].shape[1]):
		if data['matches'][0][i] == 1:
			iteration_time_matches += data['times'][0][i]
			number_of_matches += 1

	#print("matches")
	#print(iteration_time_matches)

	return iteration_time_matches/number_of_matches

print(avg_iteration_time(data_301_rays))
print(avg_matches_time(data_301_rays))

