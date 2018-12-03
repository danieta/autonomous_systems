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
        rospy.init_node('ekf_localization_node', anonymous=False)

        # number of rays used as observations (odd nr for center ray)
        self.NUMBER_OF_OBSERVATIONS = 11

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

        rot = tf.transformations.euler_from_quaternion(quat)

        #print(trans)

        # defines the distance threshold below which the robot should relocalize
        #print("check")
        #print(rospy.has_param('distance_threshold'))
        if rospy.has_param('distance_threshold'):
            self.distance_threshold = rospy.get_param('distance_threshold')
        else:
            self.distance_threshold = 1.0


        # defines the angle threshold below which the robot should relocalize
        if rospy.has_param('angle_threshold'):
            self.angle_threshold = rospy.get_param('angle_threshold')
        else:
            self.angle_threshold = 0.34906585

        # iniitialize belief of where the robot is. transpose to get a column vector
        self.current_belief = np.array(rospy.get_param('belief', [0.0, 0.0, 0.0])).T
        self.trans = np.array([self.current_belief[0], self.current_belief[1], 0])
        self.rot = np.array([0, 0, self.current_belief[2]])

         # NEED TO TWEAK THE DIAGONAL VALUES. 
        sigma = np.array(rospy.get_param('sigma', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]))
        print("SIGMA")
        print(sigma)
        self.sigma = np.reshape(sigma, (3, 3))

        # NEED TO TWEAK THE DIAGONAL VALUES. See presentation 3, slide 10
        self.R = np.array([[1,0,0], 
                           [0,1,0],
                           [0,0,1]])

        # NEED TO TWEAK THE DIAGONAL VALUES
        self.Q = np.array([[1,0], 
                           [0,1]])

        trans = self.trans - trans
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.current_belief[2]-rot[2])

        # publish the starting transformation between map and odom frame
        self.br.sendTransform((trans[0], trans[1], trans[2]),
                         (quat[0], quat[1], quat[2], quat[3]),
                         rospy.Time.now(),
                         "odom",
                         "map")


    def laser_callback(self, msg):

        #This function gets executed everytime a laser scanner msg is received on the
        #topic: /robot_0/base_scan_1

        # ============= YOUR CODE GOES HERE! =====
        # hint: msg contains the laser scanner msg
        # hint: check http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
 
        min_angle = -math.pi/2
        max_angle = -min_angle

        # calculating the indices of the first and last used ray in the laser scan message
        min_index = int((min_angle-msg.angle_min)/msg.angle_increment)
        max_index = int((max_angle-msg.angle_min)/msg.angle_increment)

        increment = (max_index - min_index)/self.NUMBER_OF_OBSERVATIONS
        indices = np.array(range(min_index, max_index, increment)).astype(int)
        angles = msg.angle_min + indices*msg.angle_increment
        distances = np.array(msg.ranges)[indices]

        #self.observation = np.hstack((distances, angles)).flatten()
        self.observation = [val for pair in zip(distances, angles) for val in pair]
        #print(self.observation)
        # ============= YOUR CODE ENDS HERE! =====


    '''
    def h(x,y,theta,x_n,y_n,theta_n):
        distance = np.sqrt( (x_n - x)**2 + (y_n - y)**2 )
        angle = np.arctan( (y_n - y) / (x_n - x) ) - theta

        return (distance, angle)
    '''

    def kalman_filter(self):
        
        
        # get the odometry
        self.listener.waitForTransform("base_link", "map", rospy.Time(), rospy.Duration(20.0))
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("base_link", "map", now, rospy.Duration(20.0))
            (current_trans,current_quat) = self.listener.lookupTransform("base_link", "map", now)
        except:
            rospy.loginfo("No odom!!!")
            current_trans = np.zeros((3,1))
            current_quat = np.array([0, 0, 0, 1.0])

        current_rot = tf.transformations.euler_from_quaternion(current_quat)

        odometry_trans = current_trans - self.trans
        odometry_rot = current_rot - self.rot

        rospy.loginfo(str(odometry_trans))
        rospy.loginfo(str(odometry_rot))

        # The distance in x and y moved, and the rotation about z
        delta_odom = np.array([odometry_trans[0],  odometry_trans[1],  odometry_rot[2]]).T

        #dont do anything if the distance traveled and angle rotated is too small
        if(np.sqrt(delta_odom[0]**2 + delta_odom[1]**2)<self.distance_threshold and delta_odom[2] < self.angle_threshold):
            return


        # delta_D_k*cos(theta_k) is the 0th element of the translation given by odometry. delta_D_k*sin(theta_k) is the 1st. 
        G = np.matrix([ [1, 0, -odometry_trans[1]], 
                        [0, 1, odometry_trans[0]], 
                        [0, 0, 1] ])

        
        

        #PREDICT
        mu_predicted = self.current_belief + delta_odom
        sigma_predicted = np.matmul( np.matmul(G, self.sigma), G.T ) + self.R #NEED TO DEFINE R, COVARIANCE OF THE STATE TRANSITION NOISE

        '''
        # UPDATE/CORRECT
        for i in range(NUMBER_OF_OBSERVATIONS): #NEED TO DEFINE NUMBER_OF_OBSERVATIONS. This should be equal to the number of rays we make
            #NEED TO DEFINE theta_n and d_n. This should be the constant angle between each ray
            H[i] = np.matrix([-np.cos(current_rot[2] + i*theta_n), -np.sin(current_rot[2] + i*theta_n), 0] 
                             [(np.sin(current_rot[2] + i*theta_n)) / d_n, -np.cos(current_rot[2] + i*theta_n) / d_n, -1])





        sigmaH = np.matmul( sigma, H.transpose )    # step 3: Kalman gain calculation (I've broken the calculation in two steps)
        K = np.matmul( sigmaH, np.linalg.inv( np.matmul( H, sigmaH ) + Q ) )
        '''     
        #update
        self.current_belief = mu_predicted      #+ np.matmul( K, z - exp_meas(predicted_state) )
        self.trans = np.array([self.current_belief[0], self.current_belief[1], 0])
        self.rot = np.array([0, 0, self.current_belief[2]]) 
        self.sigma = sigma_predicted    #np.matmul( I - np.matmul( K, H ), new_sigma )
        
        rospy.loginfo("current belief")
        rospy.loginfo(str(self.current_belief))
        rospy.loginfo("sigma")
        rospy.loginfo(str(self.sigma))

        rospy.set_param('sigma', self.sigma.flatten().tolist)
        





    
    # returns the measurement we expect to see when we're in a given state
    # here we have to do the ray tracing
    def z_exp(self, map, pose, angles):
        z_expected = np.zeros(2*len(angles)) #measurement contains distance, angle, distance, angle... must then have 2 times as many elements as 'angles' does
        for i in range(len(angles)):
            z_expected[2*i] = self.ray_trace(map, pose, angles[i], 50.0)
            z_expected[2*i+1] = angles[i]
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
            
            if map[round(curr_pixel[1]), round(curr_pixel[0])] > threshold: #if the inspected pixel is darker than a threshold returns the position of the pixel (collision detected)
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
            rospy.loginfo('Working')
            a = np.array([ 400*self.map_resolution,178*self.map_resolution,0]) # first value is column, second value is row
            b = np.array([-1.57,0,1.57])
            print(self.z_exp(self.grid_map, a, b))
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
