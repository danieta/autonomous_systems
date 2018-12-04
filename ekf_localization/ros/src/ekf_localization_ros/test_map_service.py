#!/usr/bin/env python  

import rospy
from nav_msgs.srv import GetMap

if __name__ == '__main__':
	rospy.init_node('test_map')
	rospy.wait_for_service('static_map')
	try:
		getMap = rospy.ServiceProxy('static_map', GetMap)
		occMap = getMap()
		print(occMap)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

	rospy.spin()
