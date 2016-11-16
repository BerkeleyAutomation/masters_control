'''
General filter classes for motion controls on YuMi teleop
Author: Jacky Liang
'''
from abc import ABCMeta, abstractmethod
from core import RigidTransform

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
        return pose

    def reset(self):
        return

'''
class MovingAverageFilter(MotionFilter):

class ProjectionFilter(MotionFilter):

class RotationTranslationFilter(MotionFilter):
'''
