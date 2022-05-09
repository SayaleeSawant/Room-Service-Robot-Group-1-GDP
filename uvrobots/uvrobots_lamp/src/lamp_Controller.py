#!/usr/bin/env python
import sys
import rospy
import std_msgs
import time

last_input = 0

def lamp_Switch(msg):
	global last_input

	pub = rospy.Publisher('lamp', std_msgs.msg.Bool, queue_size=10)
	
	if msg.data != last_input:
		last_input = msg.data

		if(msg.data == 0):
			pub.publish(0)
		else:
			pub.publish(1)


def lamp_Controller():
	rospy.init_node('lamp_controller', anonymous=True)
	rospy.Subscriber('lamp_switch', std_msgs.msg.Bool, lamp_Switch)
	rospy.spin()


if __name__ == "__main__":
	try:
		lamp_Controller();
	except rospy.ROSInterruptException:
		pass

