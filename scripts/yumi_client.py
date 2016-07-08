#!/usr/bin/env python
'''
Script to command YuMi by listening to dvrk masters
Author: Jacky
'''
import logging
import rospy
import numpy as np
import tfx
from geometry_msgs.msg import Pose
from time import sleep, time

from alan.control.YuMiConstants import YuMiConstants as YMC
from alan.control.YuMiRobot import YuMiRobot
from alan.constants import METERS_TO_MM

import IPython

class _RateLimiter:

    def __init__(self, period):
        self.period = period
        self.last_time = None

    @property
    def ok(self):
        if self.last_time is None:
            self.last_time = time()
            return True

        cur_time = time()
        if cur_time - self.last_time >= self.period:
            self.last_time = time()
            return True
        return False

class YuMiClient:

    _LEFT_DEL_TOPIC = '/MTML_YuMi/position_cartesian_current_delta'
    _RIGHT_DEL_TOPIC = '/MTMR_YuMi/position_cartesian_current_delta'
    _LEFT_REL_TOPIC = '/MTML_YuMi/position_cartesian_current_rel'
    _RIGHT_REL_TOPIC = '/MTMR_YuMi/position_cartesian_current_rel'

    _MASTERS_TO_YUMI = tfx.transform([[0,-1,0,0],
                                      [1,0,0,0],
                                      [0,0,1,0],
                                      [0,0,0,1]])

    _MASTERS_TO_YUMI_SCALE = 0.5
    _POSE_DIFF_THRESHOLD = 5 # this is in mm

    #TODO: Test and set this value
    _ROT_MAG_SCALE = 0

    def __init__(self, ip=YMC.IP, port_l=YMC.PORT_L, port_r=YMC.PORT_R, tcp=YMC.TCP_DEFAULT_GRIPER):
        rospy.init_node('yumi_client', anonymous=True)
        self.yumi = YuMiRobot(ip, port_l, port_r, tcp)
        sleep(1)
        self.yumi.set_z('z200')
        self.yumi.set_v(1500)
        self.yumi.reset_home()
        sleep(1)

        self.start_pose = {
            'left': self.yumi.left.get_pose(),
            'right': self.yumi.right.get_pose()
        }
        self.last_pose = {
            'left': self.yumi.left.get_pose(),
            'right': self.yumi.right.get_pose()
        }
        self.rate_limiter = {
            'left': _RateLimiter(YMC.COMM_PERIOD),
            'right': _RateLimiter(YMC.COMM_PERIOD)
        }

    def start(self):
        rospy.Subscriber(YuMiClient._LEFT_REL_TOPIC, Pose, self.left_call_back)
        rospy.Subscriber(YuMiClient._RIGHT_REL_TOPIC, Pose, self.right_call_back)
        rospy.spin()

    @staticmethod
    def _ros_to_tfx_pose(ros_pose):
        position = [ros_pose.position.x, ros_pose.position.y, ros_pose.position.z]
        rotation = [ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z, ros_pose.orientation.w]
        return tfx.pose(position, rotation)

    @staticmethod
    def _close_enough(pose1, pose2):
        tf1 = pose1.as_tf()
        tf2 = pose2.as_tf()

        delta_tf = tf1.inverse() * tf2

        diff = delta_tf.position.norm + YuMiClient._ROT_MAG_SCALE * np.linalg.norm(delta_tf.rotation.matrix)
        if diff < YuMiClient._POSE_DIFF_THRESHOLD:
            return True
        return False

    def _update_yumi(self, arm_name, ros_pose):
        rate_limiter = self.rate_limiter[arm_name]
        if not rate_limiter.ok:
            return

        arm = getattr(self.yumi, arm_name)
        yumi_start_pose = self.start_pose[arm_name]

        # turn ros pose into tfx pose 
        masters_pose = YuMiClient._ros_to_tfx_pose(ros_pose)
        
        # meters to mm
        masters_pose.position = masters_pose.position * METERS_TO_MM
        
        # scale translations
        masters_pose.position = masters_pose.position * YuMiClient._MASTERS_TO_YUMI_SCALE    

        # transform into YuMi basis
        yumi_rel_pose = YuMiClient._MASTERS_TO_YUMI * masters_pose
        
        # offset using starting pose
        yumi_pose = yumi_start_pose.as_tf() * yumi_rel_pose

        if YuMiClient._close_enough(yumi_pose, self.last_pose[arm_name]):
            return

        # updating last pose
        self.last_pose[arm_name] = yumi_pose

        # TODO: hack to only use position data for now
        yumi_pose.rotation = yumi_start_pose.rotation
        
        # send pose to YuMi
        start = time()
        arm.goto_pose(yumi_pose, wait_for_res=True)
        end = time()
        return end-start

    def left_call_back(self, ros_pose):
        start = time()
        motion_time = self._update_yumi('left', ros_pose)
        total_time = time() - start
        if motion_time is not None:
            print 'callback took {0}s'.format(total_time)
            print "cmd took {0}s".format(motion_time)
            print 'diff is {0}s'.format(total_time - motion_time)

    def right_call_back(self, ros_pose):
        #self._update_yumi('right', ros_pose)
        return

if __name__ == '__main__':
    logging.getLogger().setLevel(YMC.LOGGING_LEVEL)
    yumi_client = YuMiClient()
    yumi_client.start()