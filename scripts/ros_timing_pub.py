#!/usr/bin/python
import rospy
from time import time
from std_msgs.msg import Float64

def time_publisher():
    pub = rospy.Publisher('time_pub', Float64, queue_size=1)
    rospy.init_node('time_pub', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(time())
        rate.sleep()

if __name__ == '__main__':
    try:
        time_publisher()
    except rospy.ROSInterruptException:
        pass