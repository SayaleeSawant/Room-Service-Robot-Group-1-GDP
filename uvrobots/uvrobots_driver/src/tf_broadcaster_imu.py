#!/usr/bin/env python  
import rospy
import tf
import tf_conversions
import tf2_ros
import numpy
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


def handle_imu_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "imu_base"
    t.child_frame_id = "imu_link"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0

    #quat_msg = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]	
    #euler = tf.transformations.euler_from_quaternion(quat_msg)

    #yaw = euler[2] - 3.14159

    #q = tf.transformations.quaternion_from_euler(euler[0], euler[1], yaw)
	
    #offset_q = Quaternion(*q)
	
    #t.transform.rotation.x = offset_q.x
    #t.transform.rotation.y = offset_q.y
    #t.transform.rotation.z = offset_q.z
    #t.transform.rotation.w = offset_q.w

    t.transform.rotation.x = msg.orientation.x
    t.transform.rotation.y = msg.orientation.y
    t.transform.rotation.z = msg.orientation.z
    t.transform.rotation.w = msg.orientation.w


    br.sendTransform(t)

if __name__ == '__main__':
      rospy.init_node('tf_broadcaster_imu')
      rospy.Subscriber('/imu/data', Imu, handle_imu_pose)
      rospy.spin()

