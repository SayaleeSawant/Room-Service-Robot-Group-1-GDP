#!/usr/bin/env python
import sys
import rospy
import std_msgs
import time
import actionlib
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
# .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Dock(object):

	def __init__(self):
		self.aruco_marker_state = False
		self.aruco_marker_pos = PoseStamped()
		self.dock_state = ""
		self.aruco_clearance = 0.1

		self.ir_marker_validation = False
		self.linear_actuator = False

		self.undock_vel_msg = Twist()
		self.undock_vel_x = 0.3

		# Subscribers
		rospy.Subscriber('/aruco_single/pose', PoseStamped, self.aruco_callback)
		rospy.Subscriber('/dock_state', String, self.dock_state_callback)

		# Publishers
		self.undock_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)



	def aruco_callback(self, msg):
		rospy.loginfo('Got Aruco Marker Position %f', msg.data)
		self.aruco_marker_pos = msg.data
		self.aruco_marker_state = True

	def dock_state_callback(self, msg):
		rospy.loginfo('Dock State: ', msg.data)
		self.dock_state = msg.data

		if (self.dock_state == "DOCK"):
			if (self.aruco_marker_state):
				result = movebase_client(self)
				if result:
					self.dock_state = "DOCKED"
					rospy.loginfo("Dock Goal Exection Done!")
		elif (self.dock_state == "UNDOCK"):
			# Reverse Only
			self.undock_vel_msg.linear.x = -abs(self.uncdock_vel_x)
			self.undock_vel_msg.linear.y = 0
			self.undock_vel_msg.linear.z = 0
			self.undock_vel_msg.angular.x = 0
			self.undock_vel_msg.angular.y = 0
			self.undock_vel_msg.angular.z = 0

			while self.undock_vel_msg.get_num_connections() < 1:


			self.undock_vel_msg.publish(self.undock_vel_msg)
			

		elif (self.dock_state == "DOCKED"): 

		elif (self.dock_state == "UNDOCKED"):


	def movebase_client(self):
		if (((self.aruco_marker_pos.pose.position.z - self.aruco_clearance) != 0) or (self.aruco_marker.pos.pose.position.x != 0)):
			client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
			client.wait_for_server()

			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = "map"
			goal.target_pose.header.stamp = rospy.Time.now()
	
			goal.target_pose.pose.position.x = self.aruco_marker_pos.pose.position.z - self.aruco_clearance
			goal.target_pose.pose.position.y = self.aruco_marker_pos.pose.position.x
			goal.target_pose.pose.position.z = 0.0

			goal.target_pose.pose.orientation.x = 0.0
			goal.target_pose.pose.orientation.y = 0.0
			goal.target_pose.pose.orientation.z = 0.0
			goal.target_pose.pose.orientation.w = 1.0

			client.send_goal(goal)
			wait = client.wait_for_result()

			if not wait:
				rospy.logerr("Action server not available!")
				rospy.signal_shutdown("Action server not available!")
			else: 
				return client.get_result()


if __name__ == "__main__":
	try:
		rospy.init_node('dock_controller', anonymous=True)
		rospy.logwarn("Dock Controller Initialised")
		dock_subs = Dock()
		dock_subs.loop()
	except rospy.ROSInterruptException:
		pass
