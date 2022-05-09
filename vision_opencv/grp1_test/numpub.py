#!/usr/bin/env python2.7

#import rospy
#from std_msgs.msg import String
#pub = rospy.Publisher('chatter', String, queue_size=10)
#rospy.init_node('talker', anonymous=True)
#rate = rospy.Rate(10) # 10hz
#while not rospy.is_shutdown():
#hello_str = "hello world %s" % rospy.get_time()
#rospy.loginfo(hello_str)
#    pub.publish(b1)
#    rate.sleep()

import rospy
from std_msgs.msg import String

 
def talker():
    pub = rospy.Publisher('roomnumber', String, queue_size=4)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        file1 = open("number.txt", "r")
        numbers = file1.readlines()
        for num in numbers:
        #    rospy.loginfo(num)
            pub.publish(num)
            rate.sleep()      
        #num_str = "hello world %s" % rospy.get_time()
        file1.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
