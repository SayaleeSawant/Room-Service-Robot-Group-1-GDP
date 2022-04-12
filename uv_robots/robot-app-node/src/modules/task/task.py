import sys, select, os, subprocess
from std_msgs.msg import String, Bool
import rospy
from PIL import Image
import RPi.GPIO as GPIO

from ..nav import multi_point
from ..robot import robot
from ..feedback import feedback
from ..constants import nodes

import time


class Task:

    def __init__(self, map, map_paths, multi_point, feedback):
        # message format 
        # start/stop_service
        self.map = map
        self.map_paths = map_paths
        self.multi_point = multi_point
        self.feedback = feedback
        #actions for map
        self.GET_MAPLIST = "get_maplist"
        self.START_MAP = "start_map"
        self.STOP_MAP = "stop_map"
        self.SAVE_MAP = "save_map"
        self.DEL_MAP = "del_map"
        self.DEL_PATH = "del_path"
        self.SAVE_NEWMAP = "save_newmap"
        

        self.SAVE_MAP_PATH = "save_map_path"
        self.GET_MAP_PATH = "get_map_path"
        # dup of DEL_PATH self.DELETE_MAP_PATH = "delete_map_path"

        self.SET_DOCK = "set_dock"
        self.curr_proc = None

        #GPIO & LAPM
        self.LAMP_TOGGLE = "lamp_toggle"
        self.lamp_state = False
               
        #ROBOT process actions
        self.START_GMAPPING = "start_gmapping"
        self.START_AUTONAV = "start_autonav"
        self.CANCEL_GOAL = "cancel_goal"
        self.STOP_CURR_PROC = "stop_curr_proc"

        #MISC
        self.SET_INIT = "set_init"
        self.SET_PATH = "set_path"
        #lamp_state
        self.lamp_state_path = "/home/eaibot/dashgo_ws/src/uv_robots/robot-app-node/src/modules/lamp/lamp_state.txt"
        self.lamp_topic = rospy.Publisher('/lampState', String, queue_size=0)

        

    def callback(self, msg):
        print(msg)
        msg.data = eval(msg.data) 



        #for gmapping 
        if msg.data['name']==self.START_GMAPPING and robot.Robot.state!=robot.Robot.STATES["gmap"]:
            #self.map.stop_map()
            self.stop_curr_proc()
            self.curr_proc = subprocess.Popen(['roslaunch', 'app_node', 'gmap.launch'])
            robot.Robot.state = robot.Robot.STATES["gmap"]

        #for auto 
        #if msg.data['name'] == self.START_AUTONAV :
            #pass
            #self.stop_curr_proc()
            #self.curr_proc = subprocess.Popen(['roslaunch',  'app_node', 'auto.launch'])
            #robot.Robot.state = robot.Robot.STATES["auto"]
                
        #for map & auto nav
        if msg.data['name'] == self.START_MAP :
            self.stop_curr_proc()
            time.sleep(2)
            filename = msg.data['map_name']
            self.curr_proc = subprocess.Popen(['roslaunch',  'app_node', 'auto.launch', 'map_file:='+self.map.path+filename+'.yaml'])
            robot.Robot.state = robot.Robot.STATES["auto"]
            #self.map.start_map(filename)
                                              
        if msg.data['name'] == self.STOP_MAP :
            self.map.stop_map()

        if msg.data['name'] == self.SAVE_MAP :
            filename = msg.data['map_name'] 
            self.map.save_map(filename)

        if msg.data['name'] == self.DEL_MAP :
            filename = msg.data['map_name'] 
            self.map.del_map(filename)

        if msg.data['name'] == self.GET_MAPLIST :
            #pubish map list
            self.map.maplist_publisher()
            self.feedback.send_toast("updated map list")
            

        #for navigation
        if msg.data['name'] == self.SET_PATH:
            self.cancel_proc = subprocess.Popen(['rostopic', 'pub', '/move_base/cancel', 'actionlib_msgs/GoalID', '--', '{}{}'.format('{','}')])
            time.sleep(3)
            self.cancel_proc.terminate()
            
            self.multi_point.format_data(msg.data['data'], msg.data['isDock'])
            self.multi_point.start_nav()
            
        if msg.data['name'] == self.SET_INIT:
            robot.Robot.init_publisher(msg.data['data'])

        if msg.data['name'] == self.CANCEL_GOAL:
            print('cancelling goal....')
            self.cancel_proc = subprocess.Popen(['rostopic', 'pub', '/move_base/cancel', 'actionlib_msgs/GoalID', '--', '{}{}'.format('{','}')])
            time.sleep(3)
            self.cancel_proc.terminate()

        #misc
        if msg.data['name'] == self.STOP_CURR_PROC and self.curr_proc != None :
            self.stop_curr_proc()

        if(msg.data['name'] == self.SAVE_MAP_PATH):
            pathData = msg.data['data']
            pathName = msg.data['path_name']
            mapName = msg.data['map_name']
            self.map_paths.createPath(pathData, pathName, mapName)

        if(msg.data['name'] == self.GET_MAP_PATH):
            mapName = msg.data['map_name']
            self.map_paths.pathPublisher(mapName)

        if(msg.data['name'] == self.DEL_PATH):
            mapName = msg.data['map_name']
            pathName = msg.data['path_name']
            self.map_paths.deletePath(mapName, pathName)

        if(msg.data['name'] == self.SET_DOCK):
            dockData = msg.data['data']
            mapName = msg.data['map_name']
            self.map_paths.createDock(dockData, mapName)

        if(msg.data['name'] == self.LAMP_TOGGLE):
            lamp_topic = rospy.Publisher('/lamp_switch', Bool, queue_size=0)
            state = Bool()
            if(msg.data['state'] == "false"):
                state.data = False
                lamp_topic.publish(state)
            else:
                state.data = True
	        lamp_topic.publish(state)

        if(msg.data['name'] == self.SAVE_NEWMAP):
            fh = open(self.map.path+msg.data["map_name"]+".png", "wb")
            fh.write(msg.data['data'].decode('base64'))
            fh.close()
            Image.open(self.map.path+msg.data["map_name"]+".png").save(self.map.path+msg.data["map_name"]+".pgm")
                    
    def stop_curr_proc(self):
        try:
            self.curr_proc.terminate()

        except:
            print('no running process')

        finally:
            robot.Robot.state = None  
            self.curr_proc = None

    def key_listener(self):
        rospy.Subscriber('/key_listener', String, self.callback)
