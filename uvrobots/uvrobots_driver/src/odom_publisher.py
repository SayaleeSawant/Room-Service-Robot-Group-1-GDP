#!/usr/bin/env python
import sys
import rospy
import std_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from uvrobots_driver.msg import Encoder
import time
import numpy as np

class OdomPublisher:
    
    def __init__(self):
        self.last_input = 0
        rospy.init_node('odom_publisher', anonymous=True)
        self.pub = rospy.Publisher('odom', Odometry, queue_size=10)

        self.encoderTime = rospy.Time.now()
        self.lastEncoderTime = rospy.Time.now()
        self.prevLeftEnc = 0.0
        self.prevRightEnc = 0.0

        self.ticksPerMeter = 600 / (3.14 * 0.152) #1000 #???Enter correct tick number
        self.wheelTrack = 0.4 #1000 #???Enter correct tick number

        self.linearX = 0.0
        self.linearY = 0.0
        self.angularZ = 0.0

        # Odom Covariances 
        self.ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                                    0, 1e-3, 0, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e3]

        self.ODOM_POSE_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0,
                                    0, 1e-3, 1e-9, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e-9]

        self.ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                                    0, 1e-3, 0, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e3]

        self.ODOM_TWIST_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0,
                                    0, 1e-3, 1e-9, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e-9]






    def odom_Publisher(self, msg):
        dt = 0.0
        dleft = 0.0
        dright = 0.0
        dx = 0.0
        dy = 0.0
        dxy_ave = 0.0
        dth = 0.0
        vxy = 0.0
        vth = 0.0   

        self.encoderTime = rospy.Time.now()

        # Get the time in seconds since the last encoder measurement
        dt = (self.encoderTime - self.lastEncoderTime).to_sec()
        
        # Save the encoder time for the next calculation
        self.lastEncoderTime = self.encoderTime

        # Calculate the distance in meters traveled by the two wheels
        dleft = (msg.left_encoder - self.prevLeftEnc) / self.ticksPerMeter
        dright = (msg.right_encoder - self.prevRightEnc) / self.ticksPerMeter

        self.prevLeftEnc = msg.left_encoder
        self.prevRightEnc = msg.right_encoder

        # Compute the average linear distance over the two wheels
        dxy_ave = (dleft + dright) / 2.0

        # Compute the angle rotated
        dth = (dright - dleft) / self.wheelTrack

        # Linear velocity
        vxy = dxy_ave / dt

        # Angular velocity
        vth = dth / dt;

        # How far did we move forward
        if(dxy_ave != 0):
            dx = np.cos(dth) * dxy_ave
            dy = -np.sin(dth) * dxy_ave

            # The total distance traveled so far
            self.linearX += (np.cos(self.angularZ) * dx - np.sin(self.angularZ) * dy)
            self.linearY += (np.sin(self.angularZ) * dx + np.cos(self.angularZ) * dy)


        # The total angular displacement
        if(dth != 0):
            self.angularZ += dth

        # Represent the rotation as a quaternion
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = np.sin(self.angularZ / 2.0)
        quaternion.w = np.cos(self.angularZ / 2.0)

        # Publish the distances and speeds on the odom topic.
        # Set the timestamp to the last encoder time.
        odom_msg = Odometry()

        #???odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "odom"
        odom_msg.header.stamp = rospy.Time.now()  #???Check which time to enter here
        odom_msg.pose.pose.position.x = self.linearX
        odom_msg.pose.pose.position.y = self.linearY
        odom_msg.pose.pose.position.z = 0
        odom_msg.pose.pose.orientation = quaternion
        odom_msg.twist.twist.linear.x = vxy
        odom_msg.twist.twist.linear.y = 0
        odom_msg.twist.twist.linear.z = 0
        odom_msg.twist.twist.angular.x = 0
        odom_msg.twist.twist.angular.y = 0
        odom_msg.twist.twist.angular.z = vth

        odom_msg.pose.covariance[0] = self.ODOM_POSE_COVARIANCE[0]
        odom_msg.pose.covariance[7] = self.ODOM_POSE_COVARIANCE[7]
        odom_msg.pose.covariance[14] = self.ODOM_POSE_COVARIANCE[14]
        odom_msg.pose.covariance[21] = self.ODOM_POSE_COVARIANCE[21]
        odom_msg.pose.covariance[28] = self.ODOM_POSE_COVARIANCE[28]
        odom_msg.pose.covariance[35] = self.ODOM_POSE_COVARIANCE[35]

        odom_msg.twist.covariance[0] = self.ODOM_TWIST_COVARIANCE[0]
        odom_msg.twist.covariance[7] = self.ODOM_TWIST_COVARIANCE[7]
        odom_msg.twist.covariance[14] = self.ODOM_TWIST_COVARIANCE[14]
        odom_msg.twist.covariance[21] = self.ODOM_TWIST_COVARIANCE[21]
        odom_msg.twist.covariance[28] = self.ODOM_TWIST_COVARIANCE[28]
        odom_msg.twist.covariance[35] = self.ODOM_TWIST_COVARIANCE[35]

        self.pub.publish(odom_msg)



    def encoder_Subscriber(self):
        rospy.Subscriber('encoderLR', Encoder, self.odom_Publisher)
        rospy.spin()


if __name__ == "__main__":
    try:
        odomPublisher = OdomPublisher()
        odomPublisher.encoder_Subscriber()
    except rospy.ROSInterruptException:
        pass
