import rospy
import math

import json
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

#move this to logging classs
def write_to_log_file(message):
    #function to write the status update of the robot to a text file
    log_file = open("log_file.txt", "w")
    log_file.write(message)
    log_file.close()

#need to refactor this class to not to behave like a function
class MultiPoint():
    def __init__(self, feedback):
        self.feedback = feedback
        
    def start_nav(self):
        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        write_to_log_file("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))

        if not wait:
            write_to_log_file("Action server not available!")
            rospy.logerr("Action server not available!")
            return

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        write_to_log_file("Starting goals achievements ...")
        self.movebase_client()

    def format_data(self, data, isDock):
	self.isDock = isDock
        mobile_data = eval(data)

	    #x,y,z for each point
        # points_seq = [2.0,1.0,0,2,0.5,0,1.5,-0.5,0]
        points_seq = mobile_data['points']
        
        #yaw in degree
        yaweulerangles_seq = mobile_data['angles']
        # yaweulerangles_seq = [90,0,180]
        
        quat_seq = list()
        #Local variables of class
        self.pose_seq = list()
        self.goal_cnt = 0

        for yawangle in yaweulerangles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))

        #n=3 for x, y, z
        n = 3
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]

        for point in points:
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")
        write_to_log_file("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
        write_to_log_file("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
   
        if status == 2:
            cancelled_status = "Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!"
            self.feedback.send_toast(cancelled_status)
            rospy.loginfo(cancelled_status)
            write_to_log_file(cancelled_status)

        if status == 3:
            reached_status = "Goal pose "+str(self.goal_cnt)+" reached"
            rospy.loginfo(reached_status)
            write_to_log_file(reached_status)
            self.feedback.send_toast(reached_status)
            time.sleep(10)
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
		print(self.isDock)
		if self.isDock == 0:
                    self.feedback.send_notif()
                rospy.loginfo("Final goal pose reached!")
                write_to_log_file("Final goal pose reached!")
                return

        if status == 4:
            aborted_status = "Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server"
            self.feedback.send_toast(aborted_status)
            rospy.loginfo(aborted_status)
            write_to_log_file(aborted_status)
            return

        if status == 5:
            rejected_status = "Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server"
            self.feedback.send_toast(rejected_status)
            rospy.loginfo(rejected_status)
            write_to_log_file(rejected_status)
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")
            write_to_log_file("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        

    
