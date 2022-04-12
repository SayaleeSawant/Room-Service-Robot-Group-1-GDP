#!/usr/bin/env python
import roslib
import rospy

from sensor_msgs.msg import LaserScan

filteredScan = LaserScan()
upperAngle = 0.0
lowerAngle = 0.0
minRange = 0.0

def angularFilterIn(inputScan):
	global filteredScan, lowerAngle, upperAngle
	
	startAngle = inputScan.angle_min
	currentAngle = inputScan.angle_min
	startTime = inputScan.header.stamp

	count = 0

	for i in range(len(inputScan.ranges)):
		if startAngle < lowerAngle:
			startAngle += inputScan.angle_increment
			currentAngle += inputScan.angle_increment
			startTime += rospy.Duration(inputScan.time_increment)
		else:
			filteredScan.ranges[count] = inputScan.ranges[i]

			if len(inputScan.intensities) > i:
				filteredScan.intensities[count] = inputScan.intensities[i]
				count = count + 1

				if (currentAngle + inputScan.angle_increment) > upperAngle:
					break
			currentAngle += inputScan.angle_increment

	filteredScan.header.stamp = startTime
	filteredScan.angle_min = startAngle
	filteredScan.angle_max = currentAngle

	filteredScan.ranges = filteredScan.ranges[0:count]

	if len(inputScan.intensities) >= count:
		filteredScan.intensities = filteredScan.intensities[0:count]


def angularFilterOut(inputScan):
	global filteredScan, lowerAngle, upperAngle

	currentAngle = inputScan.angle_min
	count = 0

	for i in range(len(inputScan.ranges)):
		if (currentAngle > lowerAngle) and (currentAngle < upperAngle):
			filteredScan.ranges[i] = inputScan.range_max + 1.0
			if i < len(filteredScan.intensities):
				filteredScan.intensities[i] = 0.0;
			count = count + 1
		currentAngle += inputScan.angle_increment


def minRangeFilter(inputScan):
	global filteredScan, minRange

	for i in range(len(inputScan.ranges)):
		if (filteredScan.ranges[i] < minRange):
			filteredScan.ranges[i] = inputScan.range_max + 1.0
			filteredScan.range_min = minRange


def updateFilter(inputScan):
	global filteredScan, lowerAngle, upperAngle, minRange
	
	filteredScan = inputScan
	filteredScan.ranges = list(inputScan.ranges)
	filteredScan.intensities = list(inputScan.intensities)
	
	lowerAngle = rospy.get_param('/lidar_filter/lowerAngle')
	upperAngle = rospy.get_param('/lidar_filter/upperAngle')
	minRange = rospy.get_param('/lidar_filter/minRange')

	if(rospy.get_param('/lidar_filter/minRangeFilter')):
		minRangeFilter(inputScan)

	if(rospy.get_param('/lidar_filter/angleFilterIn')):	
		angularFilterIn(inputScan)

	if(rospy.get_param('/lidar_filter/angleFilterOut')):
		angularFilterOut(inputScan)

	pub.publish(filteredScan)


if __name__ == '__main__':
	try:
		rospy.init_node('lidar_filter', anonymous=True)
		rospy.Subscriber('/scan', LaserScan, updateFilter)
		pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=10)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass