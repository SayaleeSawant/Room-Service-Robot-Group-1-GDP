#!/usr/bin/env python
import sys
import rospy
import std_msgs
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import time

#Right
sonar0_reading = 10.0
#Left
sonar1_reading = 10.0
#Right Back
sonar2_reading = 10.0
#Left Back
sonar3_reading = 10.0

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

vel_msg = Twist()

def sonar0_reader(msg):
	global pub
	global sonar0_reading
	sonar0_reading = msg.range
	#if sonar0_reading < 7.5:
	#	vel_msg.linear.x = 0.35
	#	vel_msg.angular.z = 0.5
	#	pub.publish(vel_msg)

def sonar1_reader(msg):
	global pub
	global sonar1_reading
	sonar1_reading = msg.range
	#if sonar1_reading < 7.5:
	#	vel_msg.linear.x = 0.35
	#	vel_msg.angular.z = -0.5
	#	pub.publish(vel_msg)

def sonar2_reader(msg):
	global pub
	global sonar2_reading 
	sonar2_reading = msg.range
	#if sonar2_reading < 10.0:
	#	vel_msg.linear.x = 0.35
	#	vel_msg.angular.z = 0.0
	#	pub.publish(vel_msg)

def sonar3_reader(msg):
	global pub
	global sonar3_reading
	sonar3_reading = msg.range
	#if sonar3_reading < 10.0:
	#	vel_msg.linear.x = 0.35
	#	vel_msg.angular.z = 0.0
	#	pub.publish(vel_msg)

def collision_avoidance():
	global pub
	global vel_msg
	global sonar0_reading

	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0

	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
	
	rospy.init_node('collision_avoidance', anonymous=True)
	rospy.Subscriber('sonar0', Range, sonar0_reader)
	rospy.Subscriber('sonar1', Range, sonar1_reader)
	rospy.Subscriber('sonar2', Range, sonar2_reader)
	rospy.Subscriber('sonar3', Range, sonar3_reader)
	
	rospy.spin()


if __name__ == "__main__":
	try:
		#global pub
		#pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
		collision_avoidance();
	except rospy.ROSInterruptException:
		pass

