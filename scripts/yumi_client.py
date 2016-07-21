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
    _L_GRIPPER_POSITION  = '/dvrk/MTML/gripper_position_current'
    _R_GRIPPER_POSITION  = '/dvrk/MTMR/gripper_position_current'

    _RTF_MC_YC = RigidTransform(rotation=[[0,-1,0],
                                                                [1,0,0],
                                                                [0,0,1]],
                                from_frame='yumi_current', to_frame='masters_current')
    
    _RTF_YR_MI = RigidTransform(rotation=[[0,1,0],
                                          [-1,0,0],
                                          [0,0,1]],
                                from_frame='masters_init', to_frame='yumi_reference')

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
        self.init_pose = {
            'left': self.yumi.left.get_pose().as_frames('yumi_init', 'world'),
            'right': self.yumi.right.get_pose().as_frames('yumi_init', 'world')
        }
        self.rtf_yi_yr = {
            'left': RigidTransform(rotation=self.init_pose['left'].inverse().rotation, from_frame='yumi_reference', to_frame='yumi_init'),
            'right': RigidTransform(rotation=self.init_pose['right'].inverse().rotation, from_frame='yumi_reference', to_frame='yumi_init')
        }
        self.last_pose = {
            'left': self.init_pose['left'].copy(),
            'right': self.init_pose['right'].copy()
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
            self._l_gripper_sub.unregister()
            self._r_gripper_sub.unregister()
            self._r_motion_sub.unregister()
            self._l_motion_sub.unregister()
            
        return shutdown_hook

    def start(self):
        self._l_motion_sub = rospy.Subscriber(YuMiClient._L_REL, Pose, self._motion_callback_gen('left'))
        self._r_motion_sub = rospy.Subscriber(YuMiClient._R_REL, Pose, self._motion_callback_gen('right'))
        self._l_gripper_sub = rospy.Subscriber(YuMiClient._L_GRIPPER_CLOSE, Bool, self._gripper_callback_gen('left'))
        self._r_gripper_sub = rospy.Subscriber(YuMiClient._R_GRIPPER_CLOSE, Bool, self._gripper_callback_gen('right'))
        
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
        delta_rtf = pose1.inverse() * pose2

        diff = np.linalg.norm(delta_rtf.translation) + YuMiClient._ROT_MAG_SCALE * np.linalg.norm(delta_rtf.rotation)
        if diff < YuMiClient._POSE_DIFF_THRESHOLD:
            return True
        return False

    def _motion_callback(self, arm_name, ros_pose):
        rate_limiter = self.rate_limiter[arm_name]
        if not rate_limiter.ok:
            return

        #arm = getattr(self.yumi, arm_name)
        rtf_w_yi = self.init_pose[arm_name]

        # turn ros pose into rtf
        rtf_mi_mc = YuMiClient._ros_to_rigid_transform(ros_pose, 'masters_current', 'masters_init')
        
        # scale translations
        rtf_mi_mc.position = rtf_mi_mc.position * YuMiClient._MASTERS_TO_YUMI_SCALE

        # transform into YuMi basis
        rtf_yr_yc = YuMiClient._RTF_YR_MI * rtf_mi_mc * YuMiClient._RTF_MC_YC

        # offset using init pose
        rtf_w_yc = rtf_w_yi * self.rtf_yi_yr[arm_name] * rtf_yr_yc

        if YuMiClient._close_enough(rtf_w_yc, self.last_pose[arm_name]):
            return

        # updating last pose
        self.last_pose[arm_name] = rtf_w_yc.copy()
        
        # send pose to YuMi
        start = time()
        print rtf_w_yc.translation
        getattr(self.yumi, arm_name).goto_pose(rtf_w_yc, wait_for_res=True)
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
