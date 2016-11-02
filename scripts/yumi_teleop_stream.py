#!/usr/bin/env python
'''
Script to command YuMi by listening to dvrk masters
Author: Jacky
'''
import rospy
import numpy as np

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from masters_control.srv import *

from time import time

from core import RigidTransform
from util import T_to_ros_pose, ros_pose_to_T

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

    _L_SUB = '/yumi/masters_rel/l'
    _R_SUB = '/yumi/masters_rel/r'
    _L_PUB = '/yumi/l'
    _R_PUB = '/yumi/r'

    _T_MC_YCR = RigidTransform(rotation=[[0,-1,0],
                                        [1,0,0],
                                        [0,0,1]],
                                from_frame='yumi_current_ref', to_frame='masters_current')

    _T_YIR_MI = RigidTransform(rotation=[[0,1,0],
                                          [-1,0,0],
                                          [0,0,1]],
                                from_frame='masters_init', to_frame='yumi_init_ref')

    _MASTERS_TO_YUMI_SCALE = 1
    _POSE_DIFF_THRESHOLD = 0.001 # this is in m

    #TODO: Test and set this value
    _ROT_MAG_SCALE = 0

    def __init__(self, rate_limit=False, update_threshold_limit=False):
        self.rate_limit = rate_limit
        self.update_threshold_limit = update_threshold_limit

        if self.rate_limit:
            self.rate_limiter = {
                'left': _RateLimiter(YMC.COMM_PERIOD),
                'right': _RateLimiter(YMC.COMM_PERIOD)
            }

    def start(self):
        rospy.init_node('yumi_teleop_stream', anonymous=True)

        self.init_pose_reset_service = rospy.Service('yumi_teleop_stream_reset_init_poses', pose_str, self._reset_init_poses)

        self._r_motion_sub = rospy.Subscriber(YuMiClient._R_SUB, Pose, self._motion_callback_gen('right'))
        self._l_motion_sub = rospy.Subscriber(YuMiClient._L_SUB, Pose, self._motion_callback_gen('left'))

        self._yumi_pubs = {
            'left': rospy.Publisher(YuMiClient._L_PUB, Pose, queue_size=1),
            'right': rospy.Publisher(YuMiClient._R_PUB, Pose, queue_size=1)
        }

        rospy.on_shutdown(self._shutdown_hook_gen())
        rospy.spin()

    def _shutdown_hook_gen(self):
        def shutdown_hook():
            self._r_motion_sub.unregister()
            self._l_motion_sub.unregister()

        return shutdown_hook

    def _reset_init_poses(self, res):
        left_pose = self._ros_to_rigid_transform(res.left, 'yumi_init', 'world')
        right_pose = self._ros_to_rigid_transform(res.right, 'yumi_init', 'world')
        self.T_w_yi = {
            'left': left_pose,
            'right': right_pose
        }
        self.T_yi_yir = {
            'left': RigidTransform(rotation=self.T_w_yi['left'].inverse().rotation,
                                    from_frame='yumi_init_ref', to_frame='yumi_init'),
            'right': RigidTransform(rotation=self.T_w_yi['right'].inverse().rotation,
                                    from_frame='yumi_init_ref', to_frame='yumi_init')
        }
        self.T_ycr_yc = {
            'left': RigidTransform(rotation=self.T_w_yi['left'].rotation,
                                    from_frame='yumi_current', to_frame='yumi_current_ref'),
            'right': RigidTransform(rotation=self.T_w_yi['right'].rotation,
                                   from_frame='yumi_current', to_frame='yumi_current_ref')
        }
        self.last_T_w_yc = {
            'left': self.T_w_yi['left'].copy(),
            'right': self.T_w_yi['right'].copy()
        }

        return "ok"

    @staticmethod
    def _close_enough(pose1, pose2):
        delta_T = pose1.inverse() * pose2

        diff = np.linalg.norm(delta_T.translation) + YuMiClient._ROT_MAG_SCALE * np.linalg.norm(delta_T.rotation)
        if diff < YuMiClient._POSE_DIFF_THRESHOLD:
            return True
        return False

    def _motion_callback(self, arm_name, ros_pose):
        if self.rate_limit:
            rate_limiter = self.rate_limiter[arm_name]
            if not rate_limiter.ok:
                return

        T_w_yi = self.T_w_yi[arm_name]

        # turn ros pose into rigid transform
        T_mi_mc = ros_pose_to_T(ros_pose, 'masters_current', 'masters_init')

        # scale translations
        T_mi_mc.position = T_mi_mc.position * YuMiClient._MASTERS_TO_YUMI_SCALE

        # transform into YuMi basis
        T_yir_ycr = YuMiClient._T_YIR_MI * T_mi_mc * YuMiClient._T_MC_YCR

        # offset using init pose
        T_w_yc = T_w_yi * self.T_yi_yir[arm_name] * T_yir_ycr * self.T_ycr_yc[arm_name]

        if self.update_threshold_limit:
            if YuMiClient._close_enough(T_w_yc, self.last_T_w_yc[arm_name]):
                return

        # updating last pose
        self.last_T_w_yc[arm_name] = T_w_yc.copy()

        # publish yumi pose
        self._yumi_pubs[arm_name].publish(T_to_ros_pose(T_w_yc))

    def _motion_callback_gen(self, arm_name):
        return lambda pose: self._motion_callback(arm_name, pose)

if __name__ == '__main__':
    yumi_client = YuMiClient()
    yumi_client.start()
