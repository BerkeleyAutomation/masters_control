'''
General filter classes for motion controls on YuMi teleop
Author: Jacky Liang
'''
from abc import ABCMeta, abstractmethod
from core import RigidTransform

_TABLE_Z = 0.005

class MotionFilter:

    __metaclass__ = ABCMeta

    @abstractmethod
    def apply(self, pose):
        pass

    @abstractmethod
    def reset(self):
        pass

class IdentityFilter(MotionFilter):

    def apply(self, pose):
        pose.translation[2] = max(pose.translation[2], _TABLE_Z)
        return pose

    def reset(self):
        return

class ProjectionFilter(MotionFilter):

    def __init__(self):
        self.plane_pose = None

    def apply(self, pose):
        if self.plane_pose is None:
            self.plane_pose = pose
            return pose

        pose.translation[2] = self.plane_pose.translation[2]
        return pose

    def reset(self):
        self.plane_pose = None

class TranslationPassFilter(MotionFilter):

    def __init__(self):
        self.start_pose = None

    def apply(self, pose):
        if self.start_pose is None:
            self.start_pose = pose.copy()
            return pose

        pose.rotation = self.start_pose.rotation
        pose.translation[2] = max(pose.translation[2], _TABLE_Z)
        return pose

    def reset(self):
        self.plane_pose = None

class MovingAverageFilter(MotionFilter):

    def __init__(self, alpha):
        self.prev_pose = None
        self.alpha = alpha

    def apply(self, pose):
        if self.prev_pose == None:
            self.prev_pose = pose.copy()
            return pose

        self.prev_pose = RigidTransform.interpolate(self.prev_pose, pose, self.alpha)
        return self.prev_pose.copy()

    def reset(self):
        self.prev_pose = None
