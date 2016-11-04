#!/usr/bin/env python
"""
YuMiTeleopHost - core script that manages teleop and motion interfaces as well
as data recording for demonstrations.
Author: Jacky Liang
"""
from multiprocess import Process, Queue
import argparse
import rospy
from geometry_msgs.msg import Pose

from yumipy import YuMiRobot, YuMiSubscriber
from yumipy import YuMiConstants as ymc
from core import DataStreamRecorder, DataStreamSyncer
from perception import OpenCVCameraSensor

from masters_control.srv import str_str
#from teleop_experiment_logger import TeleopExperimentLogger
from util import T_to_ros_pose, ros_pose_to_T

import IPython

_L_SUB = "/yumi/l"
_R_SUB = "/yumi/r"

class _YuMiArmPoller(Process):

    def __init__(self, arm, pose_q, cmds_q):
        Process.__init__(self)
        self.pose_q = pose_q
        self.cmds_q = cmds_q
        self.arm = arm

        self.forward_poses = False

    def run(self):
        while True:
            if self.forward_poses and not self.pose_q.empty():
                pose = self.pose_q.get()
                res = self.arm.goto_pose(pose)
            if not self.cmds_q.empty():
                cmd = cmds_q.get()
                if cmd[0] == 'forward':
                    self.forward_poses = cmd[1]
                elif cmd[0] == 'stop':
                    break

    def stop(self):
        self.cmds_q.put(('stop',))

    def set_forward(self, val):
        self.cmds_q.put(('forward', val))

class YuMiTeleopHost:

    def __init__(self, v, z):
        self.y = YuMiRobot()
        self.y.set_v(v)
        self.y.set_z(z)
        self.y.reset_home()

        self.qs = {
            'cmds': {
                'left': Queue(),
                'right': Queue()
            },
            'poses': {
                'left': Queue(maxsize=1),
                'right': Queue(maxsize=1)
            }
        }

        self.pollers = {
            'left': _YuMiArmPoller(self.y.left, self.qs['poses']['left'], self.qs['cmds']['left']),
            'right': _YuMiArmPoller(self.y.right, self.qs['poses']['right'], self.qs['cmds']['right']),
        }

        # TODO: load actual demo names
        self._demo_names = ["demo1", "demo2", "demo3"]

    def _enqueue_pose_gen(self, q):
        def enqueue_pose(pose):
            T = ros_pose_to_T(pose)
            while q.qsize() > 0:
                q.get_nowait()
            q.put(T)
        return enqueue_pose

    def _shutdown_hook_gen(self):
        def shutdown_hook():
            for poller in self.pollers.values():
                poller.stop()
            for sub in self.subs.values():
                sub.unregister()
        return shutdown_hook

    def dispatcher(self, msg):
        rospy.loginfo("Rcv {0}".format(msg))
        transition = getattr(self, "t_{0}".format(self.cur_state))
        res = transition(msg)
        rospy.loginfo("Snt {0}".format(res))
        return res

    def run(self):
        rospy.init_node("yumi_teleop_host")
        rospy.loginfo("Init YuMiTeleopHost")

        self.subs = {
            'left': rospy.Subscriber(_L_SUB, Pose, self._enqueue_pose_gen(self.qs['poses']['left'])),
            'right': rospy.Subscriber(_R_SUB, Pose, self._enqueue_pose_gen(self.qs['poses']['right'])),
        }

        self.cur_state = 'standby'

        self.ui_service = rospy.Service('yumi_teleop_host_ui_service', str_str, self.dispatcher)

        #rospy.wait_for_service('masters_yumi_transform_reset_init_poses')
        #self.init_pose_service = rospy.ServiceProxy('masters_yumi_transform_reset_init_poses', pose_str)

        rospy.on_shutdown(self._shutdown_hook_gen())

        rospy.loginfo("Serving UI Service...")
        rospy.spin()

    def _set_poller_forwards(self, val):
        for poller in self.pollers.values():
            poller.set_forward(val)

    def _reset_masters_yumi_connector(self):
        left_pose = T_to_ros_pose(self.y.left.get_pose())
        right_pose = T_to_ros_pose(self.y.right.get_pose())

        #self.init_pose_service(left=left_pose, right=right_pose)

    def _teleop_begin(self, demo_name=False):
        self.y.reset_home()

        record = demo_name is not None
        if record:
            '''
            TODO:
            - perform setup motions
            - setup data subs/syncers
            '''
            pass

        self._reset_masters_yumi_connector()

        self._set_poller_forwards(True)

    def _teleop_pause(self):
        if self.cur_state == "teleop_record":
            # TODO: pause data recordings
            pass
        self._set_poller_forwards(False)

    def _teleop_resume(self):
        self._reset_masters_yumi_connector()
        if self.cur_state == "teleop_record_pause":
            # TODO: resume data recordings
            pass
        self._set_poller_forwards(True)

    def _teleop_finish(self):
        self._set_poller_forwards(False)
        if self.cur_state == "teleop_record":
            # TODO: save recorded data
            pass

    def t_standby(self, msg):
        if msg.req == "teleop_start":
            self._teleop_begin()
            self.cur_state = 'teleop'
        elif msg.req == "choose_demo":
            demo_name = msg.data
            self._teleop_begin(demo_name)
            self.cur_state = 'teleop_record'
        elif msg.req == "list_demos":
            res = repr(self._demo_names)
            return res
        return "ok"

    def t_teleop_record(self, msg):
        if msg.req == "teleop_pause":
            self._teleop_pause()
            self.cur_state = 'teleop_record_pause'
        elif msg.req == "teleop_finish":
            self._teleop_finish()
            self.cur_state = 'standby'
        return "ok"

    def t_teleop_record_pause(self, msg):
        if msg.req == "teleop_resume":
            self._teleop_resume()
            self.cur_state = 'teleop_record'
        elif msg.req == "teleop_finish":
            self._teleop_finish()
            self.cur_state = 'standby'
        return "ok"

    def t_teleop(self, msg):
        if msg.req == "teleop_pause":
            self._teleop_pause()
            self.cur_state = 'teleop_pause'
        elif msg.req == "teleop_finish":
            self._teleop_finish()
            self.cur_state = 'standby'
        return "ok"

    def t_teleop_pause(self, msg):
        if msg.req == "teleop_resume":
            self._teleop_resume()
            self.cur_state = 'teleop'
        elif msg.req == "teleop_finish":
            self._teleop_finish()
            self.cur_state = 'standby'
        return "ok"

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='YuMi Teleop Host')
    parser.add_argument('-z', '--zone', type=str, default='fine', help='zone settings for YuMi')
    parser.add_argument('-v', '--velocity', type=int, default=1500, help='speed settings for YuMi')
    args = parser.parse_args()

    yth = YuMiTeleopHost(args.velocity, args.zone)
    yth.run()
