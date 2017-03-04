"""
Wrapper class for teleop demonstrations
Author: Jacky
"""
import os, sys
from abc import ABCMeta, abstractmethod

_NULL = lambda *args, **kwars: None

class _PseudoYuMiArm:

    def __init__(self, arm_name, single_poller, sub_arm):
        self._arm_name = arm_name
        self._call_single_poller = single_poller
        self._sub_arm = sub_arm

    def get_state(self):
        return sub_arm.get_state()

    def get_pose(self):
        return sub_arm.get_pose()

    def goto_state(self, state):
        self._call_single_poller(self._arm_name, 'goto_state', state)

    def goto_pose(self, pose, linear=True, relative=False):
        self._call_single_poller(self._arm_name, 'goto_pose', pose, **{'lienar':linear, 'relative':relative})

    def open_gripper(self):
        self._call_single_poller(self._arm_name, 'open_gripper')

    def close_gripper(self, *args, **kwargs):
        self._call_single_poller(self._arm_name, 'close_gripper', *args, **kwargs)

    def reset_home(self):
        self._call_single_poller(self._arm_name, 'reset_home')

class _PseudoYuMiRobot:

    def __init__(self, both_poller, single_poller, sub):
        self._call_both_poller = both_poller
        self.left = _PseudoYuMiArm('left', single_poller, sub.left)
        self.right = _PseudoYuMiArm('right', single_poller, sub.right)

    def open_grippers(self):
        self._call_both_poller('open_grippers')

    def close_grippers(self):
        self._call_both_poller('close_grippers')

    def set_v(self, n):
        self._call_both_poller('set_v', n)

    def set_z(self, z):
        self._call_both_poller('set_z', z)

    def reset_home(self):
        self._call_both_poller('reset_home')

class DemoWrapper:

    __metaclass__ = ABCMeta

    def __init__(self, robot, set_filter):
        if isinstance(robot, dict):
            self.yumi = _PseudoYuMiRobot(robot['both_poller'], robot['single_poller'], robot['sub'])
        else:
            self.yumi = robot

        self.set_filter = set_filter

    @abstractmethod # this is a property when implementing
    def name(self):
        pass

    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def takedown(self):
        pass

    @staticmethod
    def load(filename, robot, set_filter=_NULL):
        dirname, basename = os.path.dirname(filename), os.path.basename(filename)
        sys.path.append(dirname)

        demo_module_name = basename[:-3]
        exec("import {0}".format(demo_module_name))
        exec("demo_class = {0}.DEMO_CLASS".format(demo_module_name))
        demo_obj = demo_class(robot, set_filter)
        return demo_obj
