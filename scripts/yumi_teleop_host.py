#!/usr/bin/env python
"""
YuMiTeleopHost - core script that manages teleop and motion interfaces as well
as data recording for demonstrations.
Author: Jacky Liang
"""
from multiprocessing import Process, Queue
import argparse
import rospy
from geometry_msgs.msg import Pose
from time import sleep
from yumipy import YuMiRobot, YuMiSubscriber, YuMiState
from yumipy import YuMiConstants as ymc
from core import DataStreamRecorder, DataStreamSyncer
from perception import OpenCVCameraSensor
from Queue import Empty
from masters_control.srv import str_str, pose_str
#from teleop_experiment_logger import TeleopExperimentLogger
from util import T_to_ros_pose, ros_pose_to_T

import IPython

_L_SUB = "/yumi/l"
_R_SUB = "/yumi/r"

class _YuMiArmPoller(Process):

    def __init__(self, pose_q, cmds_q, z, v, arm_name):
        Process.__init__(self)
        self.pose_q = pose_q
        self.cmds_q = cmds_q

        self.z = z
        self.v = v
        self.arm_name = arm_name

        self.forward_poses = False

    def run(self):
        if self.arm_name == "left":
            self.y = YuMiRobot(include_right=False)
            self.arm = self.y.left
        elif self.arm_name == "right":
            self.y = YuMiRobot(include_left=False)
            self.arm = self.y.right
        self.y.set_v(self.v)
        self.y.set_z(self.z)

        while True:
            if self.forward_poses and not self.pose_q.empty():
                try:
                    pose = self.pose_q.get()
                    res = self.arm.goto_pose(pose, relative=True)
                except Empty:
                    pass
            if not self.cmds_q.empty():
                cmd = self.cmds_q.get()
                if cmd[0] == 'forward':
                    self.forward_poses = cmd[1]
                elif cmd[0] == 'stop':
                    break
                elif cmd[0] == 'method':
                    args = cmd[3]['args']
                    kwargs = cmd[3]['kwargs']
                    method_name = cmd[2]
                    if cmd[1] == 'both':
                        method = getattr(self.y, method_name)
                    elif cmd[1] == 'single':
                        method = getattr(self.arm, method_name)
                    method(*args, **kwargs)
            sleep(0.001)

        self.y.stop()

    def stop(self):
        self.cmds_q.put(('stop',))

    def set_forward(self, val):
        self.cmds_q.put(('forward', val))

    def send_cmd(self, packet):
        self.cmds_q.put(packet)

class YuMiTeleopHost:

    def __init__(self, v, z):
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

        self.ysub = YuMiSubscriber()
        self.ysub.start()

        self.pollers = {
            'left': _YuMiArmPoller(self.qs['poses']['left'], self.qs['cmds']['left'], z, v, 'left'),
            'right': _YuMiArmPoller(self.qs['poses']['right'], self.qs['cmds']['right'], z, v, 'right'),
        }

        # TODO: load actual demo names
        self._demo_names = ["demo1", "demo2", "demo3"]

    def _enqueue_pose_gen(self, q):
        def enqueue_pose(pose):
            T = ros_pose_to_T(pose, "yumi", "world")
            while q.qsize() > 0:
                try:
                    q.get_nowait()
                except Empty:
                    pass
            q.put(T)
        return enqueue_pose

    def _shutdown_hook_gen(self):
        def shutdown_hook():
            for poller in self.pollers.values():
                poller.stop()
            for sub in self.subs.values():
                sub.unregister()
            self.ysub.stop()
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

        for poller in self.pollers.values():
            poller.start()
        self._call_both_poller('reset_home')
        self._call_both_poller('open_grippers')

        self.subs = {
            'left': rospy.Subscriber(_L_SUB, Pose, self._enqueue_pose_gen(self.qs['poses']['left'])),
            'right': rospy.Subscriber(_R_SUB, Pose, self._enqueue_pose_gen(self.qs['poses']['right'])),
        }

        self.cur_state = 'standby'

        self.ui_service = rospy.Service('yumi_teleop_host_ui_service', str_str, self.dispatcher)

        rospy.wait_for_service('masters_yumi_transform_reset_init_poses')
        self.init_pose_service = rospy.ServiceProxy('masters_yumi_transform_reset_init_poses', pose_str)

        rospy.on_shutdown(self._shutdown_hook_gen())

        rospy.loginfo("Serving UI Service...")
        rospy.spin()

    def _set_poller_forwards(self, val):
        for poller in self.pollers.values():
            poller.set_forward(val)

    def _reset_masters_yumi_connector(self):
        left_pose = T_to_ros_pose(self.ysub.left.get_pose(timestamp=False))
        right_pose = T_to_ros_pose(self.ysub.right.get_pose(timestamp=False))

        self.init_pose_service(left=left_pose, right=right_pose)

    def _call_both_poller(self, method_name, *args, **kwargs):
        for poller in self.pollers.values():
            poller.send_cmd(('method', 'both', method_name, {'args':args, 'kwargs':kwargs}))

    def _call_single_poller(self, arm_name, method_name, *args, **kwargs):
        self.pollers[arm_name].send_cmd(('method', 'single', method_name, {'args':args, 'kwargs':kwargs}))

    def _teleop_begin(self, demo_name=False):
        record = demo_name is not None
        if record:
            '''
            TODO:
            - perform setup motions
            - setup data subs/syncers
            '''
            pass

        self._call_single_poller('right', 'goto_state', YuMiState([36.42, -117.3, 35.59, 50.42, 46.19, 66.02, -100.28]))
        self._call_single_poller('left', 'goto_state', YuMiState([-36.42, -117.3, 35.59, -50.42, 46.19, 113.98, 100.28]))
        sleep(3)
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
        self._call_both_poller('reset_home')
        self._call_both_poller('open_grippers')
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
        elif msg.req == "gripper":
            self.cmd_gripper(msg.data)
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
        elif msg.req == "gripper":
            self.cmd_gripper(msg.data)
        return "ok"

    def t_teleop_pause(self, msg):
        if msg.req == "teleop_resume":
            self._teleop_resume()
            self.cur_state = 'teleop'
        elif msg.req == "teleop_finish":
            self._teleop_finish()
            self.cur_state = 'standby'
        return "ok"

    def cmd_gripper(self, data):
        cmd = eval(data)
        arm_name = cmd[0]
        if cmd[1]:
            self._call_single_poller(arm_name, 'close_gripper')
        else:
            self._call_single_poller(arm_name, 'open_gripper')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='YuMi Teleop Host')
    parser.add_argument('-z', '--zone', type=str, default='fine', help='zone settings for YuMi')
    parser.add_argument('-v', '--velocity', type=int, default=1500, help='speed settings for YuMi')
    args = parser.parse_args()

    yth = YuMiTeleopHost(args.velocity, args.zone)
    yth.run()
