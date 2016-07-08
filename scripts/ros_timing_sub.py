#!/usr/bin/python
import rospy
from time import time
from std_msgs.msg import Float64

def callback(msg):
	cur_time = time()
	rospy.loginfo('Latency is {0}s'.format(cur_time - msg.data))

def listener():
    rospy.init_node('time_sub', anonymous=True)
    rospy.Subscriber('time_pub', Float64, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
