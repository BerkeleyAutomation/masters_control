#!/usr/bin/env python
import argparse
import rospy
from geometry_msgs.msg import Pose

topic_name = None

def call_back(ros_pose):
    print topic_name, [ros_pose.position.x, ros_pose.position.y, ros_pose.position.z]

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('topic_name')
    args = parser.parse_args()
    
    rospy.init_node('temp', anonymous=True)

    TOPICS = {
        'pre_yumi': '/MTMR_YuMi/position_cartesian_current_rel',
        'master': '/dvrk/MTMR/position_cartesian_current'
    }

    topic_name = args.topic_name
    topic = TOPICS[topic_name]

    sub = rospy.Subscriber(topic, Pose, call_back)
    rospy.spin()