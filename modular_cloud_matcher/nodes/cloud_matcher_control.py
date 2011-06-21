#!/usr/bin/env python
import sys
import re
import roslib; roslib.load_manifest('modular_cloud_matcher')
import rospy
import point_cloud
from roslib.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import *
from modular_cloud_matcher.srv import *


def createCloud(fileName):
	points = []
	for line in open(fileName):
		splittedLine = re.split('(?:\s|[,]|\n)+', line.strip())
		coord = map(float, splittedLine)
		while len(coord) < 3:
			coord.append(0)
		points.append(coord)
	return point_cloud.create_cloud_xyz32(roslib.msg.Header(),points)

def icpcontrol(referenceFileName, readingsFileName):
	serviceName = '/matchClouds'
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
