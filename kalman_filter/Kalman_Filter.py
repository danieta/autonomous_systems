import numpy as np
import rospy
from std_msgs.msg import String

#H: measurement model (maps the state we're in to the measurement we expect to see)
#G: motion model
#Q: model of sensor's error (covariance matrix)
#K: Kalman gain


# TO DO: find H and G !!!!!



def kalman_filter(state, sigma, z):

	# Initialize the listener for the tf topic
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("odom", String, callback)

    #predict
    #predicted_state = something we will get from ROSARIA   #step 1: we predict the new state with odometry
    new_sigma = np.matmul( np.matmul(G, sigma), G.transpose )   #step 2: we calculate the uncertainty associated to the odometry prediction (new_sigma)

    # H = 
    sigmaH = np.matmul( sigma, H.transpose )    # step 3: Kalman gain calculation (I've broken the calculation in two steps)
    K = np.matmul( sigmaH, np.linalg.inv( np.matmul( H, sigmaH ) + Q ) )
    
    #update
    new_state = predicted_state + np.matmul( K, z - exp_meas(predicted_state) )
    new_sigma = np.matmul( I - np.matmul( K, H ), new_sigma )
    
    return new_state, new_sigma
    






# returns the measurement we expect to see when we're in a given state
# here we have to do the ray tracing
def exp_measurement (state):
    
    return z_expected
