#!/usr/bin/env python  

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('laser_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((35.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "laser",
                         "base_link")
        rate.sleep()

