'''
Crane demo file - YuMi arms are initialized in a crane-like pose
Author: Jacky
'''
from demo_wrapper import DemoWrapper
from yumipy import YuMiState
from motion_filter import IdentityFilter, ProjectionFilter, TranslationPassFilter

class TestDemo(DemoWrapper):

    @property
    def name(self):
        return 'crane_demo'

    def setup(self):
        self.yumi.left.goto_state(YuMiState([-62.77, -119.46, 53.79, 129.56, 99.18, -108.71, 42.47]))
        self.yumi.right.goto_state(YuMiState([54.77, -122.0, 51.54, 232.38, 95.3, -88.38, -42.41]))
        self.set_filter(TranslationPassFilter())

    def takedown(self):
        self.yumi.reset_home()
        self.yumi.open_grippers()

DEMO_CLASS = TestDemo
