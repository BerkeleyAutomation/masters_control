#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

class MastersSubscriber():

    def __init__(self, name):

        self.offset = None
        self.saved = np.array([0,0,0,0,0,0,0])
        self.clutch_state = False
        self.current = None
        self.pose_rel = None

        self.scaling_factor = 1

        full_ros_namespace = "/dvrk/" + name
        rospy.init_node('master_deltas',anonymous=True)
        # subscribers
        rospy.Subscriber(full_ros_namespace + '/position_cartesian_current',
                         Pose, self.__position_cartesian_current_callback)
        rospy.Subscriber('/dvrk/footpedals/clutch',
                         Bool, self.__clutch_callback)
        self.pub_rel = rospy.Publisher('/{0}_YuMi/position_cartesian_current_rel'.format(name), Pose, queue_size=10)
        self.pub_del = rospy.Publisher('/{0}_YuMi/position_cartesian_current_delta'.format(name), Pose, queue_size=10)

    def _array_to_pose(self, ary):
        point = Point(ary[0], ary[1], ary[2])
        ori = Quaternion(ary[3], ary[4], ary[5], ary[6])
        pose = Pose(point, ori)
        return pose

    def __position_cartesian_current_callback(self, msg):
        if rospy.is_shutdown():
            return
        pose_array = np.array([msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        if self.offset is None:
            self.offset = -pose_array
        if self.current is None:
            self.current = pose_array
        delta = pose_array - self.current
        self.current = pose_array

        if not self.clutch_state:
            delta_pose = self._array_to_pose(delta)
            self.pub_del.publish(delta_pose)

            self.pose_rel = self.current + self.offset
            self.pose_rel[:3] *= self.scaling_factor
            rel_pose = self._array_to_pose(self.pose_rel)
            self.pub_rel.publish(rel_pose)

    def __clutch_callback(self, msg):
        if rospy.is_shutdown():
            return
        self.clutch_state = msg.data
        if not self.clutch_state:
            self.offset = self.saved - self.current
        elif self.clutch_state:
            self.saved = self.pose_rel
            
if __name__ == "__main__":
    left = MastersSubscriber("MTML")
    right = MastersSubscriber("MTMR")
    rospy.spin()
