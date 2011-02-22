#!/usr/bin/env python
import sys
import roslib; roslib.load_manifest('modular_cloud_matcher')
import rospy
import subprocess
import time
import os
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import *

def runexperiments():
	rospy.init_node('cloud_matcher_experiments')
	# set params
	rospy.set_param('/cloud_matcher_node/startupDropCount', 0)
	rospy.set_param('/cloud_matcher_node/statFilePrefix', 'test.stats.')
	
	rospy.set_param('/cloud_matcher_node/readingDataPointsFilterCount', 2)
	rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/0/name', 'ClampOnAxisThresholdDataPointsFilter')
	rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/0/threshold', 2.5)
	rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/0/dim', 2)
	rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/1/name', 'FixstepSamplingDataPointsFilter')
	rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/1/startStep', 17.)
	rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/1/endStep', 17.)
	rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/1/stepMult', 1.)
	
	#rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/1/name', 'RandomSamplingDataPointsFilter')
	#rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/1/prob', 0.2)
	
	rospy.set_param('/cloud_matcher_node/readingStepDataPointsFilterCount', 0)
	#rospy.set_param('/cloud_matcher_node/readingStepDataPointsFilter/0/name', 'FixstepSamplingDataPointsFilter')
	#rospy.set_param('/cloud_matcher_node/readingStepDataPointsFilter/0/startStep', 111.)
	#rospy.set_param('/cloud_matcher_node/readingStepDataPointsFilter/0/endStep', 17.)
	#rospy.set_param('/cloud_matcher_node/readingStepDataPointsFilter/0/stepMult', 0.7)

	rospy.set_param('/cloud_matcher_node/keyframeDataPointsFilterCount', 2)
	rospy.set_param('/cloud_matcher_node/keyframeDataPointsFilter/0/name', 'ClampOnAxisThresholdDataPointsFilter')
	rospy.set_param('/cloud_matcher_node/keyframeDataPointsFilter/0/threshold', 2.5)
	rospy.set_param('/cloud_matcher_node/keyframeDataPointsFilter/0/dim', 2)
	rospy.set_param('/cloud_matcher_node/keyframeDataPointsFilter/1/name', 'SamplingSurfaceNormalDataPointsFilter')
	rospy.set_param('/cloud_matcher_node/keyframeDataPointsFilter/1/k', 11)

	rospy.set_param('/cloud_matcher_node/matcher/name', 'KDTreeMatcher')
	rospy.set_param('/cloud_matcher_node/matcher/epsilon', 0.0)

	rospy.set_param('/cloud_matcher_node/transformationCheckerCount', 3)  
	rospy.set_param('/cloud_matcher_node/transformationCheckers/0/name', 'ErrorTransformationChecker')
	rospy.set_param('/cloud_matcher_node/transformationCheckers/0/minRotErrorDelta', 0.001)
	rospy.set_param('/cloud_matcher_node/transformationCheckers/0/minTransErrorDelta', 0.01)
	rospy.set_param('/cloud_matcher_node/transformationCheckers/0/tail', 4)
	rospy.set_param('/cloud_matcher_node/transformationCheckers/1/name', 'CounterTransformationChecker')
	rospy.set_param('/cloud_matcher_node/transformationCheckers/1/maxIterationCount', 40)
	rospy.set_param('/cloud_matcher_node/transformationCheckers/2/name', 'BoundTransformationChecker')
	rospy.set_param('/cloud_matcher_node/transformationCheckers/2/maxRotationNorm', 1)
	rospy.set_param('/cloud_matcher_node/transformationCheckers/2/maxTranslationNorm', 0.5)

	rospy.set_param('/cloud_matcher_node/featureOutlierFilterCount', 2)
	rospy.set_param('/cloud_matcher_node/featureOutlierFilters/0/name', 'MaxDistOutlierFilter')
	rospy.set_param('/cloud_matcher_node/featureOutlierFilters/0/maxDist', .2)
	rospy.set_param('/cloud_matcher_node/featureOutlierFilters/1/name', 'MedianDistOutlierFilter')
	rospy.set_param('/cloud_matcher_node/featureOutlierFilters/1/factor', 4)

	rospy.set_param('/cloud_matcher_node/inspector/name', 'Inspector')
	rospy.set_param('/cloud_matcher_node/errorMinimizer/name', 'PointToPlaneErrorMinimizer')

	rospy.set_param('/cloud_matcher_node/sensorFrame', '/openni_rgb_optical_frame')
	rospy.set_param('/cloud_matcher_node/fixedFrame', '/rotated_world')
	rospy.set_param('/cloud_matcher_node/ratioToSwitchKeyframe', 0.7)
	
	# dump parameters
	experimentIdentifier = time.strftime('cloud_matcher_experiment.%Y-%m-%d.%H-%M-%S.')
	time.sleep(1) 
	os.system('rosparam dump ' + str(experimentIdentifier) + 'params.txt')
	
	# first dummy experiment to work around kinect startup time
	rospy.set_param('/cloud_matcher_node/statFilePrefix', '')
	tracker = subprocess.Popen(['rosrun','modular_cloud_matcher','cloud_matcher_node'])
	time.sleep(30)
	os.system('rosnode kill /cloud_matcher_node')
	tracker.wait()
	
	# run experiments epsilon
	for fixSamplingStep in range(0, 200, 1):
		# set param
		fileNamePrefix = experimentIdentifier + str(fixSamplingStep) + '.'
		rospy.set_param('/cloud_matcher_node/statFilePrefix', fileNamePrefix)
		
		#rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/1/startStep', fixSamplingStep)
		#rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/1/endStep', fixSamplingStep)
		
		#rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/1/prob', fixSamplingStep/100.0)
		
		#rospy.set_param('/cloud_matcher_node/keyframeDataPointsFilter/1/k', fixSamplingStep)
		rospy.set_param('/cloud_matcher_node/matcher/epsilon', fixSamplingStep/10.0)
		
		time.sleep(1)
		# dump parameters
		os.system('rosparam dump ' + str(fileNamePrefix) + 'params.txt')
		# run child
		tracker = subprocess.Popen(['rosrun','modular_cloud_matcher','cloud_matcher_node'])
		time.sleep(60)
		os.system('rosnode kill /cloud_matcher_node')
		tracker.wait()
	
	## run experiments fix step
	#rospy.set_param('/cloud_matcher_node/matcher/epsilon', 0.)
	#for fixSamplingStep in range(0, 200, 1):
		## set param
		#fileNamePrefix = experimentIdentifier + str(fixSamplingStep) + '.'
		#rospy.set_param('/cloud_matcher_node/statFilePrefix', fileNamePrefix)
		
		#rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/1/startStep', fixSamplingStep)
		#rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/1/endStep', fixSamplingStep)
		
		##rospy.set_param('/cloud_matcher_node/readingDataPointsFilter/1/prob', fixSamplingStep/100.0)
		
		##rospy.set_param('/cloud_matcher_node/keyframeDataPointsFilter/1/k', fixSamplingStep)
		##rospy.set_param('/cloud_matcher_node/matcher/epsilon', fixSamplingStep/100.0)
		
		#time.sleep(1)
		## dump parameters
		#os.system('rosparam dump ' + str(fileNamePrefix) + 'params.txt')
		## run child
		#tracker = subprocess.Popen(['rosrun','modular_cloud_matcher','cloud_matcher_node'])
		#time.sleep(120)
		#os.system('rosnode kill /cloud_matcher_node')
		#tracker.wait()
	
if __name__ == '__main__':
	runexperiments()
