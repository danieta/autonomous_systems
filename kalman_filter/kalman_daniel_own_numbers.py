#!/usr/bin/env python

import numpy as np
from scipy import linalg # add this to ekf_loc_node code


NUMBER_OF_RAYS = 5 #assume 5 rays

# initialize belief of where the robot is. transpose to get a column vector
self_mu = np.array([0,0,0])
self_mu = np.transpose(self_mu)

# NEED TO TWEAK THE DIAGONAL VALUES. 
self_sigma = np.array([[1,0,0], 
                       [0,1,0],
                       [0,0,1]])

# NEED TO TWEAK THE DIAGONAL VALUES. See presentation 3, slide 10
self_R = np.array([[1,0,0], 
                   [0,1,0],
                   [0,0,1]])

# NEED TO TWEAK THE DIAGONAL VALUES
self_Q = np.array([[1,0], 
                   [0,1]])

z = np.transpose(np.array([1, 2]))


def kalman_filter(*args, **kwargs):

    global self_mu
    global self_R
    global self_sigma
    global self_Q


    '''    
    # get the odometry
    (current_trans,current_rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))

    
    odometry_trans = current_trans - self.trans
    odometry_rot = current_rot - self.rot

    # The distance in x and y moved, and the rotation about z
    delta_odom = np.array([odometry_trans[0],  odometry_trans[1],  odometry_rot[2]]).T

    #dont do anything if the distance traveled and angle rotated is too small
    if(np.sqrt(delta_odom[0]**2 + delta_odom[1]**2)<self.distance_threshold and delta_odom[2] < self.angle_threshold):
        return


    # delta_D_k*cos(theta_k) is the 0th element of the translation given by odometry. delta_D_k*sin(theta_k) is the 1st. 
    G = np.matrix([ [1, 0, -odometry_trans[1]], 
                    [0, 1, odometry_trans[0] ], 
                    [0, 0,        1          ] 
                    ])
    '''


    delta_odom = np.transpose(np.array([3,  5,  0.5]))
    G = np.array([ [1,0,-5], [0,1,3], [0,0,1] ])

    
    

    #PREDICT
    ##mu_predicted = self.belief + delta_odom
    mu_predicted = self_mu
    sigma_predicted = np.matmul( np.matmul(G, self_sigma), np.transpose(G) ) + self_R #NEED TO DEFINE R, COVARIANCE OF THE STATE TRANSITION NOISE

    
    # UPDATE/CORRECT
    for i in range(NUMBER_OF_RAYS): #NEED TO DEFINE NUMBER_OF_RAYS. This should be equal to the number of rays we make at each observation
        #NEED TO DEFINE theta_n and d_n. theta_n should be the constant angle between each ray, and d_n is the length to the point we ray to

        '''
        # compute a H for every ray
        H_one_ray[i] = np.matrix([-np.cos(current_rot[2] + i*theta_n),        -np.sin(current_rot[2] + i*theta_n),        0], 
                                 [(np.sin(current_rot[2] + i*theta_n)) / d_n, -np.cos(current_rot[2] + i*theta_n) / d_n, -1])
        '''

        H_one_ray = np.array([[1,1,1], [2,2,2]])

        #z_one_ray[i] = measurement_from_ros_topic_and_we_extract_one_observation #extract d_n to get the current measurement
        # can also extract theta_n, or assume it's constant and say that our current theta_n is theta_n*i, where i is iteration number
        # assuming anyways that our z_observed is on the form: [d1, theta_1, d2, theta_2, ...]^T. One long column vector

        z_one_ray = np.transpose(np.array([5, 6]))


        # concatenate the current ray's observation matrix and observations so that we get a big H matrix that is 2i*3, and observation vector
        #  that is 2i*1 (two elements, d_i and theta_i added for every measurement)
        if (i == 0):
            H = H_one_ray
            z = z_one_ray
        else:
            H = np.concatenate((H, H_one_ray), axis=0)
            z = np.concatenate((z, z_one_ray), axis=0)

        

       

    
     
    # H is concatenated; we need to expand Q so we get a diagonal matrix with NUMBER_OF_RAYS Q's in the diagonal for the dimensions to match
    Q_expanded = linalg.block_diag(* [self_Q]*NUMBER_OF_RAYS)

    # Measurement prediction covariance
    S = np.matmul(H, np.matmul(sigma_predicted, np.transpose(H))) + Q_expanded

    # Kalman gain
    K = np.matmul(sigma_predicted, np.matmul(np.transpose(H), np.linalg.inv(S)))

    z_expected = np.repeat(3,NUMBER_OF_RAYS*2)

    # new_belief
    self_mu = mu_predicted + np.matmul(K, (z-z_expected))
    
    self_sigma = sigma_predicted - np.matmul(K, np.matmul(S, np.transpose(K)))


    print(self_sigma)


if __name__ == '__main__':
    kalman_filter()

'''
#
def expected_observations(x, y, theta, observation):
    for i in range(NUMBER_OF_RAYS):



# x, y and theta are our estimated states
def h(x, y, theta, x_n, y_n):
    distance = np.sqrt( (x_n - x)**2 + (y_n - y)**2 )
    angle = np.arctan( (y_n - y) / (x_n - x) ) - theta

    return (distance, angle)

'''