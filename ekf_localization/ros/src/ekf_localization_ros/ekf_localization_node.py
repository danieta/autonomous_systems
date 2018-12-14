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

#import matplotlib.pyplot as plt

from scipy import io as sio


# static_transform_publisher 0.0528600998223 0.0631977245212 -0.282212913036 -1.57 0 0 map mocap

class ekf_localization(object):
    '''
    Node for localizing a robot in a map based on its odometry in tf and the scan of a laser range finder
    '''
    def __init__(self):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''
        # register node in ROS network
        rospy.init_node('ekf_node', anonymous=False)

        #variables for recording the positions and variances
        self.beliefs = np.zeros((1, 3))
        self.predictions = np.zeros((1, 3))
        self.uncertainties = np.zeros((1, 3, 3))
        self.matches = [-1]
        self.times = [0]

        self.filepath = "/home/fichti/matfiles/hall_straight.mat"

        # number of rays used as observations (odd nr for center ray)
        self.NUMBER_OF_OBSERVATIONS = 51

        # position of the laser relative to base link
        self.lrf_position = np.array([0.035, 0.0, 0.0])

        # compute angles for predicted observations
        min_angle = -math.pi/2
        max_angle = -min_angle
        angle_incr = (max_angle - min_angle)/(self.NUMBER_OF_OBSERVATIONS-1)
        self.angles = np.arange(min_angle, max_angle+0.1, angle_incr)

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
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        # transform map array into 2D map matrix
        self.grid_map = np.reshape(get_map.map.data, (get_map.map.info.height, get_map.map.info.width))
        self.grid_map = (self.grid_map).T
        
        #plt.matshow(self.grid_map)
        #plt.show()

        origin = get_map.map.info.origin.position
        self.map_origin = np.array([origin.x, origin.y, origin.z])
        self.map_resolution = get_map.map.info.resolution
        self.map_width = get_map.map.info.width
        self.map_height = get_map.map.info.height

        print self.map_origin


        # create a tf listener and broadcaster instance to update tf and get positions
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        # starting point for the odometry
        
        self.listener.waitForTransform("odom", "base_link", rospy.Time(), rospy.Duration(20.0))
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("odom", "base_link", now, rospy.Duration(20.0))
            (trans,quat) = self.listener.lookupTransform("odom", "base_link", now)
        except:
            rospy.loginfo("No odom!!!")
            trans = np.zeros((3,1))
            quat = np.array([0, 0, 0, 1.0])

        self.odom_bl_trans = np.array(trans)
        self.odom_bl_rot = np.array(tf.transformations.euler_from_quaternion(quat))

        #print(trans)

        # defines the distance threshold below which the robot should relocalize
        #print("check")
        #print(rospy.has_param('distance_threshold'))
        if rospy.has_param('~distance_threshold'):
            self.distance_threshold = rospy.get_param('~distance_threshold')
        else:
            self.distance_threshold = 0.05


        # defines the angle threshold below which the robot should relocalize
        if rospy.has_param('~angle_threshold'):
            self.angle_threshold = rospy.get_param('~angle_threshold')
        else:
            self.angle_threshold = 0.05

        # iniitialize belief of where the robot is. transpose to get a column vector
        #self.current_belief = np.array(rospy.get_param('belief', [0.0, 0.0, 0.0])).T
        self.current_belief = np.array([189*self.map_resolution, 180*self.map_resolution, 1.65])
        # For room: self.current_belief = np.array([362*self.map_resolution, 149*self.map_resolution, math.pi/2])
        #self.current_belief = np.array([0.0*self.map_resolution, 0.0*self.map_resolution, 0.0])
        self.map_bl_trans = np.array([self.current_belief[0], self.current_belief[1], 0])
        self.map_bl_rot = np.array([0, 0, self.current_belief[2]])

        self.prediction_belief = self.current_belief

         # NEED TO TWEAK THE DIAGONAL VALUES. 
        self.sigma =  np.diag([1.0, 1.0, 0.3])

        # Add starting belief as first position in recording
        self.beliefs[0, :] = self.current_belief
        self.predictions[0,:] = self.current_belief
        self.uncertainties[0, :, :] = self.sigma

        self.R = np.array([0.05, 0.05, 0.05])
        self.q_obs = np.array([0.03,0.0])
        self.q_pred = np.array([math.sqrt(2)*self.map_resolution,0.01])

        self.gamma = 1.0
        self.match_fail_counter = 0


        self.map_odom_rot = np.array([0.0, 0.0, self.correct_angle(self.current_belief[2]-self.odom_bl_rot[2])])
        self.map_odom_rot_prediction = np.array([0.0, 0.0, self.correct_angle(self.current_belief[2]-self.odom_bl_rot[2])])
        self.map_odom_trans = self.map_bl_trans - self.rotate(self.odom_bl_trans, self.map_odom_rot[2])
        self.map_odom_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.map_odom_rot[2])

        # publish the starting transformation between map and odom frame
        self.br.sendTransform((self.map_odom_trans[0], self.map_odom_trans[1], self.map_odom_trans[2]),
                         (self.map_odom_quat[0], self.map_odom_quat[1], self.map_odom_quat[2], self.map_odom_quat[3]),
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
        


    def rotate(self, trans, theta):
        """
        function for turning a translation vector by the angle theta
        """
        theta = self.correct_angle(theta)
        rot_mat = np.array([[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0, 0, 1]])

        return rot_mat.dot(trans)


    def correct_angle(self, angle):
        """
        function for keeping an angle in the range of -pi to pi
        """
        while angle > math.pi:
            angle = angle - 2*math.pi

        while angle < -math.pi:
            angle = angle + 2*math.pi

        return angle



    def kalman_filter(self):
        
        start = rospy.get_time()
        # get the odometry
        self.listener.waitForTransform("odom", "base_link", rospy.Time(), rospy.Duration(20.0))
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("odom", "base_link", now, rospy.Duration(10.0))
            (current_trans,current_quat) = self.listener.lookupTransform("odom", "base_link", now)
        except:
            rospy.loginfo("No map!!!")
            current_trans = (0.0, 0.0, 0.0)
            current_quat = (0.0, 0.0, 0.0, 1.0)

        z = self.observation

        current_trans = np.array(current_trans)
        current_rot = np.array(tf.transformations.euler_from_quaternion(current_quat))

        delta_trans_odom = current_trans - self.odom_bl_trans
        delta_trans_map = self.rotate(delta_trans_odom, self.map_odom_rot[2])
        delta_trans_map_prediction = self.rotate(delta_trans_odom, self.map_odom_rot_prediction[2])
        delta_rot = np.array([0.0, 0.0, self.correct_angle(current_rot[2] - self.odom_bl_rot[2])])

        # The distance in x and y moved, and the rotation about z
        delta_odom = np.array([delta_trans_map[0],  delta_trans_map[1],  delta_rot[2]])
        delta_odom_prediction = np.array([delta_trans_map_prediction[0], delta_trans_map_prediction[1], delta_rot[2]])

        #print delta_odom

        #dont do anything if the distance traveled and angle rotated is too small
        if (np.sqrt(delta_odom[0]**2 + delta_odom[1]**2)<self.distance_threshold) and (abs(delta_odom[2]) < self.angle_threshold):
            #print("too small change. do not update odometry")
            return


        # delta_D_k*cos(theta_k) is the 0th element of the translation given by odometry. delta_D_k*sin(theta_k) is the 1st. 
        G = np.matrix([ [1, 0, -delta_odom[1]], 
                        [0, 1, delta_odom[0]], 
                        [0, 0, 1] ])

        
        

        #PREDICT
        mu_predicted = self.current_belief + delta_odom
        mu_predicted[2] = self.correct_angle(mu_predicted[2])
        sigma_predicted = np.matmul( G, np.matmul(self.sigma, G.T) ) + np.diag((delta_odom * self.R)**2) #NEED TO DEFINE R, COVARIANCE OF THE STATE TRANSITION NOISE
        self.prediction_belief = self.prediction_belief + delta_odom_prediction
        self.prediction_belief[2] = self.correct_angle(self.prediction_belief[2])

        laser_pose = mu_predicted + self.rotate(self.lrf_position, mu_predicted[2])

        # now using the predicted measurement as pose, but should actually be lasers position, not the robots position
        z_expected = self.z_exp(self.grid_map, laser_pose-self.map_origin, z[1::2])

        
        # UPDATE/CORRECT
        H_rays = np.zeros((len(z)/2,2,3))

        S_rays = np.zeros((len(z)/2, 2, 2))

        Q_unfiltered = []

        
        for i in range(len(z)/2):
            d_n = z[2*i]
            theta_n = self.correct_angle(mu_predicted[2] + z[2*i+1])
            H_rays[i] = np.array([[-np.cos(theta_n), -np.sin(theta_n), 0],
                        [np.sin(theta_n) / d_n, -np.cos(theta_n) / d_n, -1]])
            
            Q_obs = self.q_obs*d_n
            if abs(Q_obs[0]) < 0.03:
                Q_obs[0] = 0.03
            Q_ray = (self.q_pred + Q_obs)**2
            Q_unfiltered = np.hstack((Q_unfiltered, Q_ray))

            S_rays[i] = np.matmul(H_rays[i], np.matmul(sigma_predicted, H_rays[i].T)) + np.diag(Q_ray)


        indices_of_matches = []
        v_unfiltered = z-z_expected

        #print "Product"
        #MATCHING (in the middle of update step in order to remove non-matching rays)
        for i in range(len(z)/2):

            S_inv = np.linalg.inv(S_rays[i])

            v_ray = v_unfiltered[2*i:2*i+2].T

            product = v_ray.dot( S_inv.dot(v_ray.T))

            #print product

            if (product) < self.gamma:
                #print("gut")
                indices_of_matches.append(i)
            #else:
                #print("nicht gut")


        #print("successful rays")
        #print( float(len(indices_of_matches))/self.NUMBER_OF_OBSERVATIONS )

        # if the observation is not good at all, stop executing
        if len(indices_of_matches) < self.NUMBER_OF_OBSERVATIONS/3:
            print "FAILED MATCH"
            self.current_belief = mu_predicted
            self.sigma = sigma_predicted

            self.map_bl_trans = np.array([self.current_belief[0], self.current_belief[1], 0])
            self.map_bl_rot = np.array([0, 0, self.current_belief[2]])
            self.odom_bl_trans = current_trans
            self.odom_bl_rot = current_rot

            self.match_fail_counter = self.match_fail_counter + 1
            self.sigma = self.sigma*self.match_fail_counter

            if self.match_fail_counter > 10:
                print "KIDNAPPED"
                self.sigma = np.diag([100, 100, math.pi*2])


            # Update trajectory tracking
            print self.beliefs.shape
            print self.current_belief.shape
            self.beliefs = np.append(self.beliefs, np.expand_dims(self.current_belief, axis=0), axis=0) 
            self.predictions = np.append(self.predictions, np.expand_dims(self.prediction_belief, axis=0), axis=0)
            self.uncertainties = np.append(self.uncertainties, np.expand_dims(self.sigma, axis=0), axis=0)
            self.matches.append(0)
            self.times.append(rospy.get_time()-start)

            return
        else:
            print "OK MATCHING"
            self.match_fail_counter = 0


        v = []
        Q_expanded = []
        #filter out the bad matches
        for i in range(len(indices_of_matches)):
            v.append(v_unfiltered[2*indices_of_matches[i]]) 
            v.append(v_unfiltered[2*indices_of_matches[i]+1]) 
            Q_expanded.append(Q_unfiltered[2*indices_of_matches[i]])
            Q_expanded.append(Q_unfiltered[2*indices_of_matches[i]+1])
        

        H = H_rays[indices_of_matches]
        H = H.reshape(-1, H.shape[-1])    


        #UPDATE

        # expand Q so we have the 2x2 Q-matrix on the diagonal
        #Q_expanded = linalg.block_diag(* [self.Q]*(len(v)/2))


        # Measurement prediction covariance
        S = np.matmul(H, np.matmul(sigma_predicted, H.T)) + np.diag(Q_expanded)

        # Kalman gain
        K = np.matmul(sigma_predicted, np.matmul(H.T, np.linalg.inv(S)))

        delta_update = np.array(np.matmul(K, v)).flatten()
        delta_update[2] = -delta_update[2]

        # new_belief
        #new_belief = mu_predicted + np.array(np.matmul(K, v)).flatten()
        new_belief = mu_predicted + delta_update

        #update
        self.current_belief = new_belief
        self.current_belief[2] = self.correct_angle(self.current_belief[2])

        self.map_bl_trans = np.array([self.current_belief[0], self.current_belief[1], 0])
        self.map_bl_rot = np.array([0, 0, self.current_belief[2]])
        self.odom_bl_trans = current_trans
        self.odom_bl_rot = current_rot

        
        self.sigma = sigma_predicted - np.matmul( K, np.matmul( S, K.T) )

        if np.sum(np.sign(self.sigma.diagonal())-1) != 0:
            self.sigma[0, 0] = abs(self.sigma[0, 0])
            self.sigma[1, 1] = abs(self.sigma[1, 1])
            self.sigma[2, 2] = abs(self.sigma[2, 2])

        self.map_odom_rot[2] = self.correct_angle(self.map_bl_rot[2] - self.odom_bl_rot[2])
        self.map_odom_trans = self.map_bl_trans - self.rotate(self.odom_bl_trans, self.map_odom_rot[2])
        self.map_odom_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.map_odom_rot[2])
        
        rospy.loginfo("current belief")
        rospy.loginfo(str(self.current_belief))
        rospy.loginfo("sigma")
        rospy.loginfo(str(self.sigma.diagonal()))

        # Update position tracking
        self.beliefs = np.append(self.beliefs, np.expand_dims(self.current_belief, axis=0), axis=0) 
        self.predictions = np.append(self.predictions, np.expand_dims(self.prediction_belief, axis=0), axis=0)
        self.uncertainties = np.append(self.uncertainties, np.expand_dims(self.sigma, axis=0), axis=0)
        self.matches.append(1)
        self.times.append(rospy.get_time()-start)
            
        





    
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
            if map[round(curr_pixel[0]), round(curr_pixel[1])] > threshold or map[round(curr_pixel[0]), round(curr_pixel[1])] == -1: 
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

            self.kalman_filter()

            # publish transform of odom frame in map frame to tf based on EKF results
            self.br.sendTransform((self.map_odom_trans[0], self.map_odom_trans[1], self.map_odom_trans[2]),
                         (self.map_odom_quat[0], self.map_odom_quat[1], self.map_odom_quat[2], self.map_odom_quat[3]),
                         rospy.Time.now(),
                         "odom",
                         "map")

            self.br.sendTransform((self.prediction_belief[0], self.prediction_belief[1], 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, self.prediction_belief[2]), 
                                rospy.Time.now(), "prediction", "map")

            self.br.sendTransform((self.current_belief[0], self.current_belief[1], 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, self.current_belief[2]), 
                                rospy.Time.now(), "belief", "map")

            # sleep for a small amount of time
            # rospy.sleep(0.1)

        sio.savemat(self.filepath, dict([('beliefs', self.beliefs), ('predictions', self.predictions), ('sigmas', self.uncertainties), 
                                        ('times', self.times), ('matches', self.matches), ('num_of_observations', self.NUMBER_OF_OBSERVATIONS), 
                                        ('map_origin', self.map_origin), ('map_resolution', self.map_resolution)]))



if __name__ == '__main__':
     # create object of the class ekf_localization (constructor will get executed!)
     my_object = ekf_localization()
     # call run_behavior method of class EKF_localization
     my_object.run_behavior()
