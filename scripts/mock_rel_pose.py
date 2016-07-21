#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

def talker(topic_name):
    rospy.init_node('mock_master_yumi_rel_pose', anonymous=False)
    pub = rospy.Publisher(topic_name, Pose, queue_size=1)
    
    # always output same pose for debug
    origin_pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
    test_pose = Pose(Point(1,0,0), Quaternion(0,0,0,1))
    
    hz = 10
    rate = rospy.Rate(hz)
    
    print 'publishing to {0}'.format(topic_name)
    print 'publishing {0}\nat {1}hz'.format(test_pose, hz)
    
    val = 0
    while not rospy.is_shutdown():
        test_pose = Pose(Point(0,0,val), Quaternion(0,0,0,1))
        pub.publish(test_pose)
        rate.sleep()
        val += 0.001

if __name__ == '__main__':
    side = 'MTMR'
    # topic_name = '/dvrk/{0}/position_cartesian_current'.format(side)
    topic_name = '/{0}_YuMi/position_cartesian_current_rel'.format(side)
    try:
        talker(topic_name)
    except rospy.ROSInterruptException:
        pass
