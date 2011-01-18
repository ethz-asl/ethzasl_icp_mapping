#!/usr/bin/env python
import sys
import roslib; roslib.load_manifest('modular_cloud_matcher')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import *
from icpros.srv import *

def icpcontrol(referenceFileName, readingsFileName):
	rospy.init_node('cloud_matcher_control')
	rospy.wait_for_service('/msa/matchClouds')
	# read reference
	reference = PointCloud()
	for line in open(referenceFileName):
		coord = map(float, line.split())
		point = Point32()
		while len(coord) < 3:
			coord.append(0)
		point.x, point.y, point.z = coord
		reference.points.append(point)
	# read readings
	readings = PointCloud()
	for line in open(readingsFileName):
		coord = map(float, line.split())
		while len(coord) < 3:
			coord.append(0)
		point = Point32()
		point.x, point.y, point.z = coord
		readings.points.append(point)
	# call matcher
	try:
		matchClouds = rospy.ServiceProxy('/msa/matchClouds', MatchClouds)
		finalTransform = matchClouds(reference, readings)
		print finalTransform
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	
if __name__ == '__main__':
	icpcontrol(sys.argv[1], sys.argv[2])
