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
from std_msgs.msg import Bool
from time import sleep, time

from alan.control import YuMiConstants as YMC
from alan.control import YuMiRobot
from alan.core import RigidTransform

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

    _L_REL = '/MTML_YuMi/position_cartesian_current_rel'
    _R_REL = '/MTMR_YuMi/position_cartesian_current_rel'
    _L_GRIPPER_CLOSE = '/dvrk/MTML/gripper_closed_event'
    _R_GRIPPER_CLOSE = '/dvrk/MTMR/gripper_closed_event'

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

    def __init__(self, ip=YMC.IP, port_l=YMC.PORT_L, port_r=YMC.PORT_R, tcp=YMC.TCP_DEFAULT_GRIPPER):
        rospy.init_node('yumi_client', anonymous=True)
        self.yumi = YuMiRobot(ip, port_l, port_r, tcp)
        self.yumi.set_z('fine')
        self.yumi.set_v(200)
        self.yumi.reset_home()
        self.yumi.set_z('z1')
        sleep(1)
        self.T_w_yi = {
            'left': self.yumi.left.get_pose().as_frames('yumi_init', 'world'),
            'right': self.yumi.right.get_pose().as_frames('yumi_init', 'world')
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
        self.rate_limiter = {
            'left': _RateLimiter(YMC.COMM_PERIOD),
            'right': _RateLimiter(YMC.COMM_PERIOD)
        }

    def _shutdown_hook_gen(self):
        def shutdown_hook():
            rospy.loginfo("Shutting down yumi client..")
            self.yumi.stop()
            self._l_gripper_sub.unregister()
            self._r_gripper_sub.unregister()
            self._r_motion_sub.unregister()
            self._l_motion_sub.unregister()
            
        return shutdown_hook

    def start(self):
        self._r_motion_sub = rospy.Subscriber(YuMiClient._R_REL, Pose, self._motion_callback_gen('right'))
        #self._l_motion_sub = rospy.Subscriber(YuMiClient._L_REL, Pose, self._motion_callback_gen('left'))
        #self._l_gripper_sub = rospy.Subscriber(YuMiClient._L_GRIPPER_CLOSE, Bool, self._gripper_callback_gen('left'))
        #self._r_gripper_sub = rospy.Subscriber(YuMiClient._R_GRIPPER_CLOSE, Bool, self._gripper_callback_gen('right'))
        
        rospy.on_shutdown(self._shutdown_hook_gen())
        rospy.spin()

    @staticmethod
    def _ros_to_rigid_transform(ros_pose, from_frame, to_frame):
        translation = np.array([ros_pose.position.x, ros_pose.position.y, ros_pose.position.z])
        rotation = np.array([ros_pose.orientation.w, ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z])
        normed_rotation = rotation / np.linalg.norm(rotation)
        return RigidTransform(translation=translation, rotation=normed_rotation, from_frame=from_frame, to_frame=to_frame)

    @staticmethod
    def _close_enough(pose1, pose2):
        delta_T = pose1.inverse() * pose2

        diff = np.linalg.norm(delta_T.translation) + YuMiClient._ROT_MAG_SCALE * np.linalg.norm(delta_T.rotation)
        if diff < YuMiClient._POSE_DIFF_THRESHOLD:
            return True
        return False

    def _motion_callback(self, arm_name, ros_pose):
        rate_limiter = self.rate_limiter[arm_name]
        if not rate_limiter.ok:
            return

        T_w_yi = self.T_w_yi[arm_name]

        # turn ros pose into rigid transform
        T_mi_mc = YuMiClient._ros_to_rigid_transform(ros_pose, 'masters_current', 'masters_init')
        
        # scale translations
        T_mi_mc.position = T_mi_mc.position * YuMiClient._MASTERS_TO_YUMI_SCALE

        # transform into YuMi basis
        T_yir_ycr = YuMiClient._T_YIR_MI * T_mi_mc * YuMiClient._T_MC_YCR

        # offset using init pose
        T_w_yc = T_w_yi * self.T_yi_yir[arm_name] * T_yir_ycr * self.T_ycr_yc[arm_name]

        if YuMiClient._close_enough(T_w_yc, self.last_T_w_yc[arm_name]):
            return

        # updating last pose
        self.last_T_w_yc[arm_name] = T_w_yc.copy()
        
        # send pose to YuMi
        getattr(self.yumi, arm_name).goto_pose(T_w_yc)

    def _motion_callback_gen(self, arm_name):
        return lambda pose: self._motion_callback(arm_name, pose)
        
    def _gripper_callback(self, arm_name, closed):
        arm = getattr(self.yumi, arm_name)
        if closed:
            arm.close_gripper()
        else:
            arm.open_gripper()
        
    def _gripper_callback_gen(self, arm_name):
        return lambda bool: self._gripper_callback(arm_name, bool.data)

if __name__ == '__main__':
    logging.getLogger().setLevel(YMC.LOGGING_LEVEL)
    yumi_client = YuMiClient()
    yumi_client.start()
