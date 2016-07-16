#!/usr/bin/env python
'''
Script to command YuMi by listening to dvrk masters
Author: Jacky
'''
import logging
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from time import sleep, time

from alan.control.YuMiConstants import YuMiConstants as YMC
from alan.control.YuMiRobot import YuMiRobot
from alan.core import RigidTransform
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

    _LEFT_REL_TOPIC = '/MTML_YuMi/position_cartesian_current_rel'
    _RIGHT_REL_TOPIC = '/MTMR_YuMi/position_cartesian_current_rel'

    _MASTERS_TO_YUMI = RigidTransform(rotation=[[0,1,0],
                                                [-1,0,0],
                                                [0,0,1]],
                                      from_frame='masters_init', to_frame='yumi_init')

    _MASTERS_TO_YUMI_SCALE = 1
    _POSE_DIFF_THRESHOLD = 1 # this is in mm

    #TODO: Test and set this value
    _ROT_MAG_SCALE = 0

    def __init__(self, ip=YMC.IP, port_l=YMC.PORT_L, port_r=YMC.PORT_R, tcp=YMC.TCP_DEFAULT_GRIPER):
        rospy.init_node('yumi_client', anonymous=True)
        self.yumi = YuMiRobot(ip, port_l, port_r, tcp)
        self.yumi.set_z('z1')
        self.yumi.set_v(200)
        self.yumi.reset_home()
        sleep(2)
        self.init_pose = {
            'left': RigidTransform(translation=self.yumi.left.get_pose().translation, from_frame='yumi_init', to_frame='world'),
            'right': RigidTransform(translation=self.yumi.right.get_pose().translation, from_frame='yumi_init', to_frame='world')
        }
        self.last_pose = {
            'left': self.init_pose['left'],
            'right': self.init_pose['right']
        }
        self.rate_limiter = {
            'left': _RateLimiter(YMC.COMM_PERIOD),
            'right': _RateLimiter(YMC.COMM_PERIOD)
        }
        self.left_time = time()
        self.left_count = 0

    def _shutdown_hook_gen(self):
        def shutdown_hook():
            rospy.loginfo("Shutting down yumi client..")
            self.yumi.stop()
            self._left_sub.unregister()
            self._right_sub.unregister()
        return shutdown_hook

    def start(self):
        self._left_sub = rospy.Subscriber(YuMiClient._LEFT_REL_TOPIC, Pose, self.left_call_back)
        self._right_sub = rospy.Subscriber(YuMiClient._RIGHT_REL_TOPIC, Pose, self.right_call_back)
        rospy.on_shutdown(self._shutdown_hook_gen())
        rospy.spin()

    @staticmethod
    def _ros_to_rigid_transform(ros_pose):
        translation = np.array([ros_pose.position.x, ros_pose.position.y, ros_pose.position.z])
        rotation = np.array([ros_pose.orientation.w, ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z])
        normed_rotation = rotation / np.linalg.norm(rotation)
        return RigidTransform(translation=translation, rotation=normed_rotation, from_frame='yumi', to_frame='masters_init')

    @staticmethod
    def _close_enough(pose1, pose2):
        delta_rtf = pose1.inverse() * pose2

        diff = np.linalg.norm(delta_rtf.translation) + YuMiClient._ROT_MAG_SCALE * np.linalg.norm(delta_rtf.rotation)
        if diff < YuMiClient._POSE_DIFF_THRESHOLD:
            return True
        return False

    def _update_yumi(self, arm_name, ros_pose):
        rate_limiter = self.rate_limiter[arm_name]
        if not rate_limiter.ok:
            return

        arm = getattr(self.yumi, arm_name)
        yumi_init_pose = self.init_pose[arm_name]

        # turn ros pose into rtf
        masters_rel_pose = YuMiClient._ros_to_rigid_transform(ros_pose)
        
        # meters to mm, and scale translations
        masters_rel_pose.position = masters_rel_pose.position * METERS_TO_MM
        masters_rel_pose.position = masters_rel_pose.position * YuMiClient._MASTERS_TO_YUMI_SCALE

        # transform into YuMi basis
        yumi_rel_pose = YuMiClient._MASTERS_TO_YUMI * masters_rel_pose

        # offset using init pose
        yumi_pose = yumi_init_pose * yumi_rel_pose

        if YuMiClient._close_enough(yumi_pose, self.last_pose[arm_name]):
            return

        # updating last pose
        self.last_pose[arm_name] = yumi_pose
        
        # send pose to YuMi
        start = time()
        arm.goto_pose(yumi_pose, wait_for_res=True)
        end = time()

        self.left_count += 1
        if arm_name == 'left':
            cur = time()
            period = cur - self.left_time
            if period > 1:
                print "Sent {0} msgs in {1}s. About {2}hz".format(self.left_count, period, self.left_count /1./ period)
                self.left_count = 0
                self.left_time = time()

        return end-start

    def left_call_back(self, ros_pose):
        return
        start = time()
        motion_time = self._update_yumi('left', ros_pose)
        total_time = time() - start
        if motion_time is not None:
            return
            print 'LEFT'
            print 'callback took {0}s'.format(total_time)
            print "cmd took {0}s".format(motion_time)
            print 'diff is {0}s'.format(total_time - motion_time)

    def right_call_back(self, ros_pose):
        start = time()
        motion_time = self._update_yumi('right', ros_pose)
        total_time = time() - start
        if motion_time is not None:
            return
            print 'RIGHT'
            print 'callback took {0}s'.format(total_time)
            print "cmd took {0}s".format(motion_time)
            print 'diff is {0}s'.format(total_time - motion_time)


if __name__ == '__main__':
    logging.getLogger().setLevel(YMC.LOGGING_LEVEL)
    yumi_client = YuMiClient()
    yumi_client.start()
