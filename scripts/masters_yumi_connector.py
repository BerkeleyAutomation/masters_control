#!/usr/bin/env python
"""
MastersYuMiConnector that subscribers to poses of masters and publishes corresponding
target poses for the YuMi.
Meant to operate along yumi_teleop_client and yumi_teleop_host
Author: Jacky Liang
"""
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from time import time
from masters_control.srv import pose_str

from core import RigidTransform
from util import T_to_ros_pose, ros_pose_to_T

_T_MC_YCR = RigidTransform(rotation=[[0,-1,0],
                                        [1,0,0],
                                        [0,0,1]],
                                from_frame='yumi_current_ref', to_frame='masters_current')

_T_YIR_MI = RigidTransform(rotation=[[0,1,0],
                                      [-1,0,0],
                                      [0,0,1]],
                            from_frame='masters_init', to_frame='yumi_init_ref')

class MastersYuMiConnector:

    def __init__(self, name):
        self.clutch_state = False
        self._clutch_i = 0

        # masters reference poses
        self.T_mz_cu_t = RigidTransform(from_frame=self._clutch('up'), to_frame='masters_zero')
        self.T_w_cu_t = None
        self.T_mzr_mz = None
        self.T_mc_mcr = None
        self.T_w_mc = None
        self.T_w_cd_t1 = None

        # yumi reference poses
        self.T_w_yi = None
        self.T_yi_yir = None
        self.T_ycr_yc = None

        full_ros_namespace = "/dvrk/" + name
        rospy.init_node('master_yumi_connector',anonymous=True)
        rospy.loginfo("Initializing node")

        # subscribing to clutch and rel pose
        rospy.loginfo("Subscribing to position_cartesian_current for {0}".format(name))
        self.rel_pose_sub = rospy.Subscriber('{0}/position_cartesian_current'.format(full_ros_namespace),
                         Pose, self._position_cartesian_current_callback)

        rospy.loginfo("Subscribing to clutch for {0}".format(name))
        self.clutch_sub = rospy.Subscriber('/dvrk/footpedals/clutch', Bool, self._clutch_callback)

        # publishing to /yumi/r or /yumi/l
        self.pub_name = '/yumi/{0}'.format(name[-1].lower())
        self.pub = rospy.Publisher(self.pub_name, Pose, queue_size=1)

        self.has_zeroed = False
        self.has_clutched_down = False
        self.has_clutched_up = False

        rospy.loginfo("Waiting for first resest init pose...")

    def _reset_init_poses(self, yumi_pose):
        rospy.loginfo("Reset Init Pose for {0}".format(self.pub_name))
        self.has_zeroed = True

        self.T_w_cu_t = self.T_w_mc.as_frames(self._clutch('up'), 'world')
        self.T_mzr_mz = RigidTransform(rotation=self.T_w_cu_t.rotation,
                                        from_frame='masters_zero', to_frame='masters_zero_ref')
        self.T_mc_mcr = RigidTransform(rotation=self.T_w_cu_t.inverse().rotation,
                                        from_frame='masters_current_ref', to_frame='masters_current')

        self.T_w_yi = yumi_pose.copy()
        self.T_yi_yir = RigidTransform(rotation=self.T_w_yi.inverse().rotation, from_frame='yumi_init_ref', to_frame='yumi_init')
        self.T_ycr_yc = RigidTransform(rotation=self.T_w_yi.rotation, from_frame='yumi_current', to_frame='yumi_current_ref')
        rospy.loginfo("Done!")

    def _clutch(self, state):
        return 'clutch_{0}_{1}'.format(state, self._clutch_i)

    def _position_cartesian_current_callback(self, ros_pose):
        if rospy.is_shutdown():
            return

        self.T_w_mc = ros_pose_to_T(ros_pose, 'masters_current', 'world')

        if not self.has_zeroed:
            return

        # only update YuMi if clutch is not pressed
        if not self.clutch_state:
            T_mz_mc = self.T_mz_cu_t * self.T_w_cu_t.inverse() * self.T_w_mc
            T_mzr_mcr = self.T_mzr_mz * T_mz_mc * self.T_mc_mcr

            T_mi_mc = T_mzr_mcr.as_frames("masters_current", "masters_init")
            T_yir_ycr = _T_YIR_MI * T_mi_mc * _T_MC_YCR
            T_w_yc = self.T_w_yi * self.T_yi_yir * T_yir_ycr * self.T_ycr_yc

            self.pub.publish(T_to_ros_pose(T_w_yc))

    def _clutch_callback(self, msg):
        if rospy.is_shutdown():
            return

        cluch_down = msg.data

        if cluch_down:
            self._clutch_i += 1
            rospy.loginfo("Got clutch down: {0}".format(self._clutch_i))
            # clutch down
            self.T_w_cd_t1 = self.T_w_mc.as_frames(self._clutch('down'), 'world')
        else:
            # clutch up
            rospy.loginfo("Updating cluch up for {0}".format(self._clutch_i))
            self.T_mz_cu_t = self.T_mz_cu_t * self.T_w_cu_t.inverse() * \
                               self.T_w_cd_t1 * RigidTransform(from_frame=self._clutch('up'), to_frame=self._clutch('down'))

            # updating last known cluch up pose
            self.T_w_cu_t = self.T_w_mc.as_frames(self._clutch('up'), 'world')

        self.clutch_state = cluch_down

    def shutdown(self):
        self.rel_pose_sub.unregister()
        self.clutch_sub.unregister()

if __name__ == "__main__":
    left = MastersYuMiConnector("MTML")
    right = MastersYuMiConnector("MTMR")

    def reset_init_poses(res):
        left._reset_init_poses(ros_pose_to_T(res.left, 'yumi_init', 'world'))
        right._reset_init_poses(ros_pose_to_T(res.right, 'yumi_init', 'world'))
        return 'ok'

    init_pose_reset_service = rospy.Service('masters_yumi_transform_reset_init_poses', pose_str, reset_init_poses)

    def shutdown_hook():
        left.shutdown()
        right.shutdown()
    rospy.on_shutdown(shutdown_hook)

    rospy.spin()
