#!/usr/bin/env python
'''
Script to command YuMi by listening to dvrk masters
Author: Jacky
'''
import logging
import rospy
import numpy as np
from geometry_msgs.msg import Pose

from alan.control.YuMiConstants import YuMiConstants as YMC
from alan.control.YuMiRobot import YuMiRobot
from alan.constants import METERS_TO_MM

_LEFT_DEL_TOPIC = '/MTML_YuMi/position_cartesian_current_delta'
_RIGHT_DEL_TOPIC = '/MTMR_YuMi/position_cartesian_current_delta'
_LEFT_REL_TOPIC = '/MTML_YuMi/position_cartesian_current_rel'
_RIGHT_REL_TOPIC = '/MTMR_YuMi/position_cartesian_current_rel'

class YuMiClient:

    def __init__(self, ip=YMC.IP, port_l=YMC.PORT_L, port_r=YMC.PORT_R, tcp=YMC.TCP_DEFAULT_GRIPER):
        print 'creating yumi'
        self.yumi = YuMiRobot(ip, port_l, port_r, tcp)
        
    def _callback(self, arm, pose):
        pos = np.array([pose.position.x, pose.position.y, pose.position.z]) * METERS_TO_MM
        arm.goto_pose_delta(pos, wait_for_res=False)

    def left_call_back(self, pose):
        self._callback(self.yumi.left, pose)

    def right_call_back(self, pose):
        self._callback(self.yumi.right, pose)
        
def listener():
    rospy.init_node('yumi_client', anonymous=True)

    yumi_client = YuMiClient()
    rospy.Subscriber(_LEFT_DEL_TOPIC, Pose, yumi_client.left_call_back)
    rospy.Subscriber(_RIGHT_DEL_TOPIC, Pose, yumi_client.right_call_back)
    rospy.spin()

if __name__ == '__main__':
    logging.getLogger().setLevel(YMC.LOGGING_LEVEL)
    listener()
