#!/usr/bin/env python
import rospy
import sys

from modules.robot import robot
from modules.map import map, path_manager
from modules.nav import multi_point
from modules.task import task
from modules.feedback import feedback
from modules.constants import nodes

from std_msgs.msg import String
from pathlib import Path

import geometry_msgs
import tf, time

class Node:
    def __init__(self):    
        self.PATH_LOC = '/home/uvrobots-nano/dashgo_ws/src/uv_robots/robot-app-node/src/modules/map/paths/'
        self.MAP_LOC = '/home/uvrobots-nano/dashgo_ws/src/uvrobots/uvrobots_nav/maps/'
    
        self.map = map.Map(self.MAP_LOC, 'pgm', "*.pgm")
        self.map_paths = path_manager.PathManager(self.PATH_LOC)
        self.feedback = feedback.Feedback();
        self.multi_point = multi_point.MultiPoint(self.feedback)
        self.task = task.Task(self.map, self.map_paths, self.multi_point, self.feedback)
        self.ros_init()
        

    def ros_init(self):
        log_publisher = rospy.Publisher('/log', String, queue_size=10)

        rospy.init_node('app_node', anonymous=True)
        rate = rospy.Rate(1)

        #init_listeners
        self.task.key_listener()
        self.map.map_listener()
        if(not self.map.map_metadata):
            print("waiting for map server cant publish odom")

        while( not map.Map.map_metadata and not rospy.is_shutdown()):
            rate.sleep()

        robot_in = robot.Robot()
        robot_in.odom_listener()
        robot_in.pose_listener()
        robot_in.init_listener() 

        while not rospy.is_shutdown():
            robot_in.odom_publisher()
            self.feedback.monitor_node(nodes.AMCL)
            #read from log file and publish to /log topic
            #log_file_path = Path(__file__).parent / "modules/nav/log_file.txt"
            #log_message = open(log_file_path, "r").read()
            #log_publisher.publish(log_message)
            rate.sleep()

        rospy.spin()
    

if __name__ == "__main__":
    Node()
    feedback.Feedback.node_status = { nodes.AMCL : False }
