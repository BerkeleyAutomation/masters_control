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
        self.output = None

        self.scaling_factor = 1

        full_ros_namespace = "/dvrk/" + name

        rospy.init_node('master_deltas',anonymous = True)
        # subscribers
        rospy.Subscriber(full_ros_namespace + '/position_cartesian_current',
                         Pose, self.__position_cartesian_current_callback)
        rospy.Subscriber('/dvrk/footpedals/clutch',
                         Bool, self.__clutch_callback)
        self.pub = rospy.Publisher('/' + name + '_deltas/position_cartesian_current', Pose, queue_size=10)

    def __position_cartesian_current_callback(self, msg):
        if rospy.is_shutdown():
            return
        if self.offset == None:
            self.offset = -np.array([msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.current = np.array([msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        if not self.clutch_state:
            self.output = self.current + self.offset
            self.final_output = self.output * self.scaling_factor
            point = Point(self.final_output[0], self.final_output[1], self.final_output[2])
            orientation = Quaternion(self.output[3], self.output[4], self.output[5], self.output[6])
            pose = Pose(point, orientation)
            self.pub.publish(pose)

        

    def __clutch_callback(self, msg):
        if rospy.is_shutdown():
            return
        self.clutch_state = msg.data
        if not self.clutch_state:
            self.offset = self.saved - self.current
        elif self.clutch_state:
            self.saved = self.output
            
if __name__ == "__main__":
    left = MastersSubscriber("MTML")
    right = MastersSubscriber("MTMR")
    rospy.spin()