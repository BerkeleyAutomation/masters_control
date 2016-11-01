_L_GRIPPER_CLOSE = '/dvrk/MTML/gripper_closed_event'
_R_GRIPPER_CLOSE = '/dvrk/MTMR/gripper_closed_event'

self._r_gripper_sub = rospy.Subscriber(YuMiClient._R_GRIPPER_CLOSE, Bool, self._gripper_callback_gen('right'))
self._l_gripper_sub = rospy.Subscriber(YuMiClient._L_GRIPPER_CLOSE, Bool, self._gripper_callback_gen('left'))

    def _gripper_callback(self, arm_name, closed):
        arm = getattr(self.yumi, arm_name)
        if closed:
            arm.close_gripper()
        else:
            arm.open_gripper()

    def _gripper_callback_gen(self, arm_name):
        return lambda bool: self._gripper_callback(arm_name, bool.data)


        self._l_gripper_sub.unregister()
        self._r_gripper_sub.unregister()

from multiprocessing import Process, Queue

class _YuMiArmPoller(Process):

    def __init__(self, arm, pose_q, times_q):
        Process.__init__(self)
        self.pose_q = pose_q
        self.arm = arm

    def run(self):
        while True:
            if not self.pose_q.empty():
                pose = self.pose_q.get()
                if pose is None:
                    break
                res = self.arm.goto_pose(pose)
