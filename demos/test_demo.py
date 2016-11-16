'''
Example demo file
Author: Jacky
'''
from demo_wrapper import DemoWrapper
from yumipy import YuMiState
from motion_filter import IdentityFilter, MovingAverageFilter

class TestDemo(DemoWrapper):

    @property
    def name(self):
        return 'test_demo'

    def setup(self):
        self.yumi.left.goto_state(YuMiState([-36.42, -117.3, 35.59, -50.42, 46.19, 113.98, 100.28]))
        self.yumi.right.goto_state(YuMiState([36.42, -117.3, 35.59, 50.42, 46.19, 66.02, -100.28]))
        #self.set_filter(MovingAverageFilter(0.5))
        self.set_filter(IdentityFilter())

    def takedown(self):
        self.yumi.left.goto_state(YuMiState([-36.42, -117.3, 35.59, -50.42, 46.19, 113.98, 100.28]))
        self.yumi.right.goto_state(YuMiState([36.42, -117.3, 35.59, 50.42, 46.19, 66.02, -100.28]))
        self.yumi.close_grippers()

DEMO_CLASS = TestDemo
