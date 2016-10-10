#!/usr/bin/env python
import rospy
import argparse
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

def talker(topic_name, direction, hz, delta_val, s):
    rospy.init_node('mock_master_yumi_rel_pose', anonymous=False)
    pub = rospy.Publisher(topic_name, Pose, queue_size=1)
    
    # always output same pose for debug
    origin_pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
    test_pose = Pose(Point(1,0,0), Quaternion(0,0,0,1))
    
    rate = rospy.Rate(hz)
    
    print 'publishing to {0}'.format(topic_name)
    print 'publishing {0}\nat {1}hz'.format(test_pose, hz)
    
    val = delta_val
    while not rospy.is_shutdown():
        if direction == 'x':
            translation = Point(val,0,0)
        if direction == 'y':
            translation = Point(0,val,0)
        if direction == 'z':
            translation = Point(0,0,val)

        test_pose = Pose(translation, Quaternion(0,0,0,1))

        print 'snt: ', [test_pose.position.x, test_pose.position.y, test_pose.position.z]
        pub.publish(test_pose)
        rate.sleep()

        if s == 'f':
            val += delta_val
        elif s == 'o':
            val *= -1

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Mock rel pose for masters teleop')
    parser.add_argument('-a', '--arm', type=str, default='l', help='arm')
    parser.add_argument('-d', '--direction', type=str, default='y', help='direction (axis)')
    parser.add_argument('-f', '--frequency', type=float, default=30, help='freq of update')
    parser.add_argument('-i', '--increment', type=float, default=0.001, help='increment each update')
    parser.add_argument('-m', '--mode', type=str, default='masters', help='fake masters or fake intermediatery')
    parser.add_argument('-s', '--s', type=str, default='f', help='[f]orward in direction or [o]scillate in direction')
    args = parser.parse_args()

    if args.arm == 'l':
        side = 'MTML'
    else:
        side = 'MTMR'

    if args.arm == 'masters':        
        topic_name = '/dvrk/{0}/position_cartesian_current_mock'.format(side)
    else:
        topic_name = '/{0}_YuMi/position_cartesian_current_rel'.format(side)
    
    try:
        talker(topic_name, args.direction, args.frequency, args.increment, args.s)
    except rospy.ROSInterruptException:
        pass
