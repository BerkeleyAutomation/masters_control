#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Transform

def transform_publisher(name, transform):
    """
    Creates a transform publisher that publishes the Ros Transform transform with the node called transforms/name.
    """
    pub = rospy.Publisher('transforms/' + name, Transform, queue_size=10)
    rospy.init_node('transform_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing: {0}".format(transform))
        pub.publish(transform)
        rate.sleep()

if __name__ == '__main__':
    try:
        quaternion = [1,0,0,0]
        translation = [0,0,0]
        quaternion = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        pose = Vector3(translation[0], translation[1], translation[2])
        transform = Transform(pose, quaternion)
        
        transform_publisher("masters_to_YuMi", transform)

    except rospy.ROSInterruptException:
        pass
