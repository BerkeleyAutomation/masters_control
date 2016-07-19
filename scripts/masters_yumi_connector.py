#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from alan.core import RigidTransform

class MastersYuMiConnector:

    def __init__(self, name):
        self.clutch_state = False
        self._clutch_i = 0

        self.rtf_mr_cu_t = RigidTransform(from_frame=self._clutch('up'), to_frame='masters_reference')
        self.rtf_w_cu_t = None
        self.rtf_w_mc = None
        self.rtf_w_cd_t1 = None        

        full_ros_namespace = "/dvrk/" + name
        rospy.loginfo("Initializing node")
        rospy.init_node('master_yumi_rel_pose',anonymous=True)
        
        # subscribers
        rospy.loginfo("Subscribing to position_cartesian_current for {0}".format(name))
        rospy.Subscriber(full_ros_namespace + '/position_cartesian_current',
                         Pose, self._position_cartesian_current_callback)
        
        rospy.loginfo("Subscribing to clutch for {0}".format(name))
        rospy.Subscriber('/dvrk/footpedals/clutch', Bool, self._clutch_callback)

        rospy.loginfo("Publishing relative position_cartesian_current for {0}".format(name))
        self.pub_rel = rospy.Publisher('/{0}_YuMi/position_cartesian_current_rel'.format(name), Pose, queue_size=1)

    def _ros_pose_to_rtf(self, ros_pose, from_frame, to_frame):
        translation = [ros_pose.position.x, ros_pose.position.y, ros_pose.position.z]
        rotation = [ros_pose.orientation.w, ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z]

        rtf = RigidTransform(rotation=rotation, translation=translation, from_frame=from_frame, to_frame=to_frame)
        return rtf

    def _rtf_to_ros_pose(self, rtf):
        point = Point(rtf.translation[0], rtf.translation[1], rtf.translation[2])
        rtf_quat = rtf.quaternion
        quaternion = Quaternion(rtf_quat[1], rtf_quat[2], rtf_quat[3], rtf_quat[0])
        ros_pose = Pose(point, quaternion)
        return ros_pose

    def _clutch(self, state):
        return 'clutch_{0}_{1}'.format(state, self._clutch_i)

    def _position_cartesian_current_callback(self, ros_pose):
        if rospy.is_shutdown():
            return

        self.rtf_w_mc = self._ros_pose_to_rtf(ros_pose, 'masters_current', 'world')

        # initialize rtf_w_cu_t to the pose the masters started off with
        if self.rtf_w_cu_t is None:
            self.rtf_w_cu_t = self.rtf_w_mc.as_frames(self._clutch('up'), 'world')

        # only update YuMi if clutch is not pressed
        if not self.clutch_state:
            rtf_mr_mc = self.rtf_mr_cu_t * self.rtf_w_cu_t.inverse() * self.rtf_w_mc

            pose_mr_mc = self._rtf_to_ros_pose(rtf_mr_mc)
            self.pub_rel.publish(pose_mr_mc)

    def _clutch_callback(self, msg):
        if rospy.is_shutdown():
            return

        cluch_down = msg.data

        if cluch_down:
            self._clutch_i += 1
            # clutch down
            self.rtf_w_cd_t1 = self.rtf_w_mc.as_frames(self._clutch('down'), 'world')
        else:
            # clutch up
            print "Updating cluch up for {0}".format(self._clutch_i)
            self.rtf_mr_cu_t = self.rtf_mr_cu_t * self.rtf_w_cu_t.inverse() * \
                               self.rtf_w_cd_t1 * RigidTransform(from_frame=self._clutch('up'), to_frame=self._clutch('down'))
            
            # updating last known cluch up pose
            self.rtf_w_cu_t = self.rtf_w_mc.as_frames(self._clutch('up'), 'world')

        self.clutch_state = cluch_down

if __name__ == "__main__":
    #left = MastersYuMiConnector("MTML")
    right = MastersYuMiConnector("MTMR")
    rospy.spin()
