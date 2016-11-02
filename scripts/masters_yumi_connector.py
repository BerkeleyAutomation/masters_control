#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from time import time

from alan.core import RigidTransform
from util import T_to_ros_pose, ros_pose_to_T

import IPython

class MastersYuMiConnector:

    def __init__(self, name):
        self.clutch_state = False
        self._clutch_i = 0

        self.T_mz_cu_t = RigidTransform(from_frame=self._clutch('up'), to_frame='masters_zero')
        self.T_w_cu_t = None
        self.T_mzr_mz = None
        self.T_mc_mcr = None
        self.T_w_mc = None
        self.T_w_cd_t1 = None

        full_ros_namespace = "/dvrk/" + name
        rospy.loginfo("Initializing node")
        rospy.init_node('master_yumi_rel_pose',anonymous=True)

        # subscribers
        rospy.loginfo("Subscribing to position_cartesian_current for {0}".format(name))
        rospy.Subscriber('{0}/position_cartesian_current'.format(full_ros_namespace),
                         Pose, self._position_cartesian_current_callback)

        rospy.loginfo("Subscribing to clutch for {0}".format(name))
        rospy.Subscriber('/dvrk/footpedals/clutch', Bool, self._clutch_callback)

        rospy.loginfo("Publishing relative position_cartesian_current for {0}".format(name))
        self.pub_rel = rospy.Publisher('/yumi/masters_rel/{0}'.format(name[-1].lower()), Pose, queue_size=1)

        self.has_zeroed = False

        self.started = time()
        self.has_clutched_down = False
        self.has_clutched_up = False

    def _reset_init_poses(self):
        self.T_w_cu_t = self.T_w_mc.as_frames(self._clutch('up'), 'world')
        self.T_mzr_mz = RigidTransform(rotation=self.T_w_cu_t.rotation,
                                        from_frame='masters_zero', to_frame='masters_zero_ref')
        self.T_mc_mcr = RigidTransform(rotation=self.T_w_cu_t.inverse().rotation,
                                        from_frame='masters_current_ref', to_frame='masters_current')

    def _clutch(self, state):
        return 'clutch_{0}_{1}'.format(state, self._clutch_i)

    def _position_cartesian_current_callback(self, ros_pose):
        if rospy.is_shutdown():
            return

        rospy.loginfo("got: {0}".format([ros_pose.position.x, ros_pose.position.y, ros_pose.position.z]))
        self.T_w_mc = ros_pose_to_T(ros_pose, 'masters_current', 'world')

        # initialize T_w_cu_t to the pose the masters started off with
        if not self.has_zeroed:
            self.has_zeroed = True
            self._reset_init_poses("")

        # only update YuMi if clutch is not pressed
        if not self.clutch_state:
            T_mz_mc = self.T_mz_cu_t * self.T_w_cu_t.inverse() * self.T_w_mc

            T_mzr_mcr = self.T_mzr_mz * T_mz_mc * self.T_mc_mcr

            pose_mzr_mcr = T_to_ros_pose(T_mzr_mcr)

            rospy.loginfo('snt: {0}'.format()[pose_mzr_mcr.position.x, pose_mzr_mcr.position.y, pose_mzr_mcr.position.z])
            self.pub_rel.publish(pose_mzr_mcr)

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

if __name__ == "__main__":
    left = MastersYuMiConnector("MTML")
    right = MastersYuMiConnector("MTMR")

    def reset_init_poses(args):
        left._reset_init_poses()
        right._reset_init_poses()
        return 'ok'

    init_pose_reset_service = rospy.Service('masters_yumi_connector_reset_init_poses', generic_msg, reset_init_poses)
    rospy.spin()
