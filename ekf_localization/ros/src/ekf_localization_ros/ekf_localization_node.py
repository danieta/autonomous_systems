#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ROS python api with lots of handy ROS functions
import rospy

# to be able to get the current frames and positions
import tf

# to be able to subcribe to laser scanner data
from sensor_msgs.msg import LaserScan

# to be able to publish Twist data (and move the robot)
from geometry_msgs.msg import Twist

# to be able to get the map
from nav_msgs.msg import OccupancyGrid

# to be able to obtain the map
from nav_msgs.srv import GetMap

# to be able to do matrix multiplications
import numpy as np

# to be able to do calculations for ray tracing
import math

import matplotlib.pyplot as plt

from scipy import linalg


class ekf_localization(object):
    '''
    Exposes a behavior for the pioneer robot so that moves forward until
    it has an obstacle at 1.0 m then stops rotates for some time to the
    right and resumes motion.
    '''
    def __init__(self):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''
        # register node in ROS network
        rospy.init_node('ekf_node', anonymous=False)

        # number of rays used as observations (odd nr for center ray)
        self.NUMBER_OF_OBSERVATIONS = 5

        # compute angles for predicted observations
        min_angle = -math.pi/2
        max_angle = -min_angle
        angle_incr = (max_angle - min_angle)/(self.NUMBER_OF_OBSERVATIONS-1)
        self.angles = np.arange(min_angle, max_angle+0.1, angle_incr)

        print(self.angles)

        # print message in terminal
        rospy.loginfo('ekf localization started !')
        # subscribe to pioneer laser scanner topic
        if rospy.has_param('laser_topic'):
            # retrieves the name of the LaserScan topic from the parameter server if it exists
            rospy.Subscriber(rospy.get_param('laser_topic'), LaserScan, self.laser_callback)
        else:
            rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        # obtain currently published map
        rospy.wait_for_service('static_map')
        try:
            getMap = rospy.ServiceProxy('static_map', GetMap)
            get_map = getMap()
            # DEBUG: print(self.grid_map)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        self.grid_map = np.reshape(get_map.map.data, (get_map.map.info.height, get_map.map.info.width))

        self.grid_map = (self.grid_map).T
        
        #plt.matshow(self.grid_map)
        #plt.show()

        self.map_resolution = get_map.map.info.resolution
        self.map_width = get_map.map.info.width
        self.map_height = get_map.map.info.height


        # create a tf listener and broadcaster instance to update tf and get positions
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        # starting point for the odometry
        
        self.listener.waitForTransform("base_link", "odom", rospy.Time(), rospy.Duration(20.0))
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("base_link", "odom", now, rospy.Duration(20.0))
            (trans,quat) = self.listener.lookupTransform("base_link", "odom", now)
        except:
            rospy.loginfo("No odom!!!")
            trans = np.zeros((3,1))
            quat = np.array([0, 0, 0, 1.0])

        self.odom_trans = np.array(trans)
        self.odom_rot = np.array(tf.transformations.euler_from_quaternion(quat))

        #print(trans)

        # defines the distance threshold below which the robot should relocalize
        #print("check")
        #print(rospy.has_param('distance_threshold'))
        if rospy.has_param('~distance_threshold'):
            self.distance_threshold = rospy.get_param('~distance_threshold')
        else:
            self.distance_threshold = 0.1

        print(self.distance_threshold)


        # defines the angle threshold below which the robot should relocalize
        if rospy.has_param('~angle_threshold'):
            self.angle_threshold = rospy.get_param('~angle_threshold')
        else:
            self.angle_threshold = 0.34906585

        # iniitialize belief of where the robot is. transpose to get a column vector
        #self.current_belief = np.array(rospy.get_param('belief', [0.0, 0.0, 0.0])).T
        self.current_belief = np.array([195*self.map_resolution, 180*self.map_resolution, math.pi/2])
        self.trans = np.array([self.current_belief[0], self.current_belief[1], 0])
        self.rot = np.array([0, 0, self.current_belief[2]])

         # NEED TO TWEAK THE DIAGONAL VALUES. 
        self.sigma =  np.diag([0.5, 0.5, 0.15])

        self.R = np.diag([0.001, 0.001, 0.001])
        self.Q = np.diag([1,1])

        self.gamma = 1


        map_trans = self.trans - self.odom_trans
        map_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.current_belief[2]-self.odom_rot[2])

        # publish the starting transformation between map and odom frame
        self.br.sendTransform((map_trans[0], map_trans[1], map_trans[2]),
                         (map_quat[0], map_quat[1], map_quat[2], map_quat[3]),
                         rospy.Time.now(),
                         "odom",
                         "map")


    def laser_callback(self, msg):

        min_angle = -math.pi/2
        max_angle = -min_angle

        # calculating the indices of the first and last used ray in the laser scan message
        min_index = int((min_angle-msg.angle_min)/msg.angle_increment)
        max_index = int((max_angle-msg.angle_min)/msg.angle_increment)

        increment = (max_index - min_index)/(self.NUMBER_OF_OBSERVATIONS-1)
        indices = np.array(range(min_index, max_index+1, increment)).astype(int)
        angles = msg.angle_min + indices*msg.angle_increment
        distances = np.array(msg.ranges)[indices]
        


        # Remove distances that are NaN, and the corresponding angles
        indices = np.isfinite(distances)
        angles = angles[indices]
        distances = distances[indices]

        self.observation = [val for pair in zip(distances, angles) for val in pair]
        


    '''
    def h(x,y,theta,x_n,y_n,theta_n):
        distance = np.sqrt( (x_n - x)**2 + (y_n - y)**2 )
        angle = np.arctan( (y_n - y) / (x_n - x) ) - theta

        return (distance, angle)
    '''

    def kalman_filter(self):
        
        
        # get the odometry
        self.listener.waitForTransform("base_link", "odom", rospy.Time(), rospy.Duration(20.0))
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("base_link", "odom", now, rospy.Duration(10.0))
            (current_trans,current_quat) = self.listener.lookupTransform("base_link", "odom", now)
        except:
            rospy.loginfo("No map!!!")
            current_trans = np.zeros((3,1))
            current_quat = np.array([0, 0, 0, 1.0])

        z = self.observation

        current_rot = tf.transformations.euler_from_quaternion(current_quat)

        delta_trans = np.array(current_trans) - self.odom_trans
        delta_rot = np.array(current_rot) - self.odom_rot

        # The distance in x and y moved, and the rotation about z
        delta_odom = np.array([delta_trans[0],  delta_trans[1],  delta_rot[2]])

        print delta_odom

        #dont do anything if the distance traveled and angle rotated is too small
        if (np.sqrt(delta_odom[0]**2 + delta_odom[1]**2)<self.distance_threshold) and (delta_odom[2] < self.angle_threshold):
            #print("too small change. do not update odometry")
            return


        # delta_D_k*cos(theta_k) is the 0th element of the translation given by odometry. delta_D_k*sin(theta_k) is the 1st. 
        G = np.matrix([ [1, 0, -delta_odom[1]], 
                        [0, 1, delta_odom[0]], 
                        [0, 0, 1] ])

        
        

        #PREDICT
        mu_predicted = self.current_belief + delta_odom
        sigma_predicted = np.matmul( np.matmul(G, self.sigma), G.T ) + self.R #NEED TO DEFINE R, COVARIANCE OF THE STATE TRANSITION NOISE

        # now using the predicted measurement as pose, but should actually be lasers position, not the robots position
        z_expected = self.z_exp(self.grid_map, mu_predicted, z[1::2])

        
        # UPDATE/CORRECT
        H_rays = np.zeros((len(z)/2,2,3))

        S_rays = np.zeros((len(z)/2, 2, 2))

        
        for i in range(len(z)/2):
            d_n = z[2*i]
            theta_n = z[2*i+1]
            H_rays[i] = np.array([[-np.cos(current_rot[2] + theta_n), -np.sin(current_rot[2] + theta_n), 0],
                        [(np.sin(current_rot[2] + theta_n)) / d_n, -np.cos(current_rot[2] + theta_n) / d_n, -1]])

            

            S_rays[i] = np.matmul(H_rays[i], np.matmul(sigma_predicted, H_rays[i].T)) + self.Q


        indices_of_matches = []
        v_unfiltered = z-z_expected
        #MATCHING (in the middle of update step in order to remove)
        for i in range(len(z)/2):

            S_inv = np.linalg.inv(S_rays[i])

            v_ray = v_unfiltered[2*i:2*i+2].T

            print v_ray.shape

            product = v_ray.dot( S_inv.dot(v_ray.T))

            if (product) < self.gamma:
                print("gut")
                indices_of_matches.append(i)
            else:
                print("nicht gut")

        print(v_unfiltered)
        print(indices_of_matches)

        v = []
        #filter out the bad matches
        for i in range(len(indices_of_matches)):
            v.append(v_unfiltered[2*indices_of_matches[i]]) 
            v.append(v_unfiltered[2*indices_of_matches[i]+1]) 
        
        S = S_rays[indices_of_matches]
        H = H_rays[indices_of_matches]

        H = H.reshape(-1, H.shape[-1])

        print(v)
        print "H"
        print(H)

        """
        if i==0:
            H = H_one_ray
        else:
            H = np.concatenate((H, H_one_ray))
        """
        



        #UPDATE

        # expand Q so we have the 2x2 Q-matrix on the diagonal
        Q_expanded = linalg.block_diag(* [self.Q]*(len(self.observation)/2))

        # Measurement prediction covariance
        S = np.matmul(H, np.matmul(sigma_predicted, H.T)) + Q_expanded

        # Kalman gain
        K = np.matmul(sigma_predicted, np.matmul(H.T, np.linalg.inv(S)))

        # new_belief
        new_belief = mu_predicted + np.matmul(K, (z-z_expected))

        #print(new_belief)

        
            
        

        


        #update
        self.current_belief = mu_predicted      #+ np.matmul( K, z - exp_meas(predicted_state) )
        self.trans = np.array([self.current_belief[0], self.current_belief[1], 0])
        self.rot = np.array([0, 0, self.current_belief[2]])
        self.odom_trans = np.array(current_trans)
        self.odom_rot = np.array(current_rot)
        
        self.sigma = sigma_predicted    #np.matmul( I - np.matmul( K, H ), new_sigma )

        map_trans = self.trans - self.odom_trans
        map_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.current_belief[2]-self.odom_rot[2])

        # publish the starting transformation between map and odom frame
        self.br.sendTransform((map_trans[0], map_trans[1], map_trans[2]),
                         (map_quat[0], map_quat[1], map_quat[2], map_quat[3]),
                         rospy.Time.now(),
                         "odom",
                         "map")
        
        rospy.loginfo("current belief")
        rospy.loginfo(str(self.current_belief))
        rospy.loginfo("sigma")
        rospy.loginfo(str(self.sigma))

        rospy.set_param('sigma', self.sigma.flatten().tolist)
        





    
    # returns the measurement we expect to see when we're in a given state
    # here we have to do the ray tracing
    def z_exp(self, map, pose, angles):
        z_expected = np.zeros(2*len(angles)) #measurement contains distance, angle, distance, angle... must then have 2 times as many elements as 'angles' does
        for i, theta in enumerate(angles):
            z_expected[2*i] = self.ray_trace(map, pose, theta, 50.0)
            z_expected[2*i+1] = theta
        return z_expected

    
    def ray_trace(self, map, pose, angle, threshold):
        #computes ray_angle from robot_angle and relative_angle
        ray_angle = pose[2] + angle
        
        #brings the ray_angle in the range 0-2pi
        ray_angle = ray_angle%(2*math.pi)
        if ray_angle < 0:
            ray_angle = 2 * math.pi - ray_angle
        
        # finds tan and cotan of the ray angle
        tan = math.tan(ray_angle)
        if tan != 0:
            cotan = 1/tan
        
        map_height = map.shape[0]
        map_width = map.shape[1]
        
        start_pixel = [round(pose[0]/self.map_resolution), round(pose[1]/self.map_resolution)]
        curr_pixel = [round(pose[0]/self.map_resolution), round(pose[1]/self.map_resolution)]
        
        # DEBUG: map.flags.writeable = True
        #DEBUG: print("ray_angle:", ray_angle/math.pi*180,"tan:", tan)
        #       image.flags.writeable = True
        
        #while we are within the image boundaries
        while curr_pixel[0] < map_height-1 and curr_pixel[0] >= 0 and curr_pixel[1] < map_width-1 and curr_pixel[1] >= 0:
            #if the inspected pixel is darker than a threshold returns the position of the pixel (collision detected)
            if map[round(curr_pixel[0]), round(curr_pixel[1])] > threshold: 
                #print(self.map_resolution*math.sqrt((curr_pixel[0]-start_pixel[0])**2 + (curr_pixel[1]-start_pixel[1])**2))
                return self.map_resolution*math.sqrt((curr_pixel[0]-start_pixel[0])**2 + (curr_pixel[1]-start_pixel[1])**2) #euclidean distance
        
            #map[map_height-round(curr_pixel[1])-1,round(curr_pixel[0])] = 0 # sets the inspected pixel to black (for debugging)
            
        #finds the next pixel to inspect:
            #if ray_angle is between -45 and +45 (included)
            if (ray_angle >= math.pi*7/4 or ray_angle <= math.pi/4):
                curr_pixel[0] += 1  #increment x
                curr_pixel[1] += tan  #increment y according to ray_angle
            
            #if ray_angle is between +45 and +135 (not included)
            elif (ray_angle > math.pi/4 and ray_angle < math.pi*3/4):
                curr_pixel[0] += cotan
                curr_pixel[1] += 1 #increment y
                
            #if ray_angle is between 135 and 225 degrees (included)
            elif (ray_angle >= math.pi*3/4 and ray_angle <= math.pi*5/4):
                curr_pixel[0] -= 1
                curr_pixel[1] -= tan
            
            #if ray_angle is between 225 and 315 degrees (not included)
            elif (ray_angle > math.pi*5/4 and ray_angle < math.pi*7/4):
                curr_pixel[0] -= cotan
                curr_pixel[1] -= 1
            
            else:
                return None

        return None
    



    def run_behavior(self):
        rospy.loginfo('Working')
        while not rospy.is_shutdown():
            #rospy.loginfo('Working')
            #a = np.array([ 400*self.map_resolution,178*self.map_resolution,0]) # first value is column, second value is row
            #b = np.array([-1.57,0,1.57])
            #print(self.z_exp(self.grid_map, a, b))
            self.kalman_filter()

            # sleep for a small amount of time
            rospy.sleep(0.1)



#def main():
#    print('Hello')
#    # create object of the class ekf_localization (constructor will get executed!)
#    my_object = ekf_localization()
#    # call run_behavior method of class EKF_localization
#    my_object.run_behavior()

if __name__ == '__main__':
     # create object of the class ekf_localization (constructor will get executed!)
     my_object = ekf_localization()
     # call run_behavior method of class EKF_localization
     my_object.run_behavior()
