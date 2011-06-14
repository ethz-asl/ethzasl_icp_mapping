#!/usr/bin/env python
import sys
import roslib; roslib.load_manifest('modular_cloud_matcher')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import *
from modular_cloud_matcher.srv import *
import point_cloud

def createCloud(fileName):
	points = []
	for line in open(fileName):
		coord = map(float, line.split())
		point = Point32()
		while len(coord) < 3:
			coord.append(0)
		point.x, point.y, point.z = coord
		points.append(point)
	return point_cloud.create_cloud_xyz32(Header(),pts)

def icpcontrol(referenceFileName, readingsFileName):
	serviceName = '/modular_cloud_matcher/matchClouds'
	rospy.init_node('cloud_matcher_control')
	rospy.wait_for_service(serviceName)
	# read reference
	reference = createCloud(referenceFileName)
	# read readings
	readings = createCloud(readingsFileName)
	# call matcher
	try:
		matchClouds = rospy.ServiceProxy(serviceName, MatchClouds)
		finalTransform = matchClouds(reference, readings)
		print finalTransform
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	
if __name__ == '__main__':
	if len(sys.argv) != 3:
		print 'Usage: ' + sys.argv[0] + ' reference reading'
		sys.exit()
	icpcontrol(sys.argv[1], sys.argv[2])
