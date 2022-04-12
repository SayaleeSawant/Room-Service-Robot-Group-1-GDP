import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose

import math
import json

from ..map import map

import tf, geometry_msgs



class Robot:
    state = None
    STATES = { 'gmap':'gmap',  'auto':'auto'}
    
    init_pos = None #robot is at 0,0,0

    def __init__(self):
        self.odom_data = '{"x":0 ,"y":0,"angle":0}'
        self.pose_data = '{"x":0 ,"y":0,"angle":0}'
        self.listener = tf.TransformListener()

    def get_rotation (self, orientation_q):
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        #print(yaw, math.degrees(yaw))
        return (math.degrees(-yaw))

    def pose_callback(self, od):
        if not map.Map.map_metadata:
             return
        _x = od.position.x/map.Map.map_metadata.resolution/map.Map.map_metadata.width
        _y = od.position.y/map.Map.map_metadata.resolution/map.Map.map_metadata.height
        self.pose_data = '{"x":' + '{}'.format(_x) + ',"y":' + '{}'.format(_y) + ',"angle":' + '{}'.format(self.get_rotation(od.orientation))+'}'

    def odom_callback(self, od):
        if not map.Map.map_metadata:
             return
        '''
        self.listener.waitForTransform("/map", "/odom", rospy.Time(0), rospy.Duration(3.0));
        (trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))'''
        _x = od.pose.pose.position.x/map.Map.map_metadata.resolution/map.Map.map_metadata.width
        _y = od.pose.pose.position.y/map.Map.map_metadata.resolution/map.Map.map_metadata.height
        self.odom_data = '{"x":' + '{}'.format(round(_x,2)) + ',"y":' + '{}'.format(round(_y,2)) + ',"angle":' + '{}'.format(self.get_rotation(od.pose.pose.orientation))+'}'

    def pose_listener(self):
        rospy.Subscriber('/robot_pose', Pose, self.pose_callback)
        
    def odom_listener(self):
        rospy.Subscriber('/odom_combined', Odometry, self.odom_callback)
    

    def odom_publisher(self):    
        odom_pub = rospy.Publisher('/odom_op', String, queue_size=0)
        
        #if(Robot.state == Robot.STATES['gmap']):
        #   odom_pub.publish(self.pose_data)
        #else:
        odom_pub.publish(self.pose_data)
        

    @staticmethod
    def init_publisher(init_pos, convert=True):                        
        init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        if not convert:
            print(init_pos)
            init_pub.publish(init_pos)
            return
        
        quat_orientations = quaternion_from_euler(0, 0, init_pos['angle']*(-math.pi/180), axes='sxyz')
        initpose_msg = PoseWithCovarianceStamped()
        initpose_msg.header.frame_id = "map"
        initpose_msg.pose.pose.position.x = init_pos['x']
        initpose_msg.pose.pose.position.y = init_pos['y']
        initpose_msg.pose.pose.orientation.x = quat_orientations[0]
        initpose_msg.pose.pose.orientation.y = quat_orientations[1]
        initpose_msg.pose.pose.orientation.z = quat_orientations[2]
        initpose_msg.pose.pose.orientation.w = quat_orientations[3]
        init_pub.publish(initpose_msg)
    
    def init_callback(self, init_pos):
        Robot.init_pos = init_pos
        rospy.loginfo( "INITIAL POSITION SET :)" )
        
    def init_listener(self):
        rospy.loginfo('listening for initial position...')
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_callback)
