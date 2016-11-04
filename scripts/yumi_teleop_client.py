#!/usr/bin/env python
"""
Script to be ran on the 'client' machine on a masters-YuMi teleop setup. Client
machine is the computer connected to the masters' vision system.
Author: Jacky Liang
"""
#import rospy
import cv2
import numpy as np
import rospy
from multiprocess import Process, Queue

from std_msgs.msg import Bool
from masters_control.srv import str_str
from util import str_str_service_wrapper

class UI(Process):

    BG_COLOR = (0,0,255)
    TXT_COLOR = (0,0,0)
    HI_COLOR = (0,255,255)

    def __init__(self, cid):
        Process.__init__(self)
        self.req_q = Queue()
        self.res_q = Queue()
        self.cid = cid

    def gen_list_view(self, frame):
        overlay = frame.copy()

        o_top_left = np.array([20,20])
        o_bottom_right = np.array([200, 60])
        o_text_left = np.array([30, 50])

        delta = np.array([0,50])

        for i, item in enumerate(self.list_view):
            top_left = tuple(o_top_left + i * delta)
            bottom_right = tuple(o_bottom_right + i * delta)
            text_left = tuple(o_text_left + i * delta)

            rec_color = UI.HI_COLOR if i == self.list_index else UI.BG_COLOR

            cv2.rectangle(overlay, top_left, bottom_right, rec_color,-1)
            cv2.putText(overlay, item, text_left, cv2.FONT_HERSHEY_SIMPLEX, 0.7, UI.TXT_COLOR, 1)

        cv2.putText(overlay, "AUTOLab YuMi Teleop Interface", (20, overlay.shape[0]-10), cv2.FONT_HERSHEY_PLAIN, 1.0, (255,255,255), 1)

        return overlay

    def run(self):
        self.cam = cv2.VideoCapture(self.cid)
        self.left = cv2.namedWindow("left")
        self.right = cv2.namedWindow("right")

        self.list_view = []
        self.list_index = 0
        self.show_overlay = True
        while True:
            _, frame = self.cam.read()

            if not self.req_q.empty():
                req = self.req_q.get()
                if req[0] == "stop":
                    break
                elif req[0] == "list_view":
                    self.list_view = req[1]
                    self.list_index = 0
                elif req[0] == "overlay":
                    self.show_overlay = req[1]

            if self.show_overlay and len(self.list_view) > 0:
                overlay = self.gen_list_view(frame)
                cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

            cv2.imshow('left', frame)
            cv2.imshow('right', frame)

            # TODO: Replace w/ ROS subscribers
            pressed = cv2.waitKey(1) & 0xFF
            if self.show_overlay:
                if pressed == ord('w'):
                    self.list_index -= 1
                elif pressed == ord('s'):
                    self.list_index += 1
                elif pressed == ord('d'):
                    self.res_q.put(self.list_view[self.list_index])
                    self.list_view = []
                if len(self.list_view) > 0:
                    self.list_index = self.list_index % len(self.list_view)
            if pressed == ord('a'):
                self.show_overlay = not self.show_overlay

        self.cam.release()
        cv2.destroyAllWindows()

    def list_view(self, lst):
        self.req_q.put(("list_view", lst))
        while self.res_q.empty():
            pass
        return self.res_q.get()

    def set_overlay(self, overlay):
        self.req_q.put(("overlay", overlay))

    def stop(self):
        self.req_q.put(("stop",))

class YuMiTeleopClient:

    def __init__(self, cid):
        self.menu_main = ("Collect Demos", "Sandbox", "Quit")
        self.menu_pause_teleop = ("Pause", "Finish")
        self.menu_resume_teleop = ("Resume", "Finish")
        self.menu_demo = None

        self.ui = UI(cid)

        rospy.init_node("yumi_teleop_client")
        rospy.loginfo("Init YuMiTeleopClient")

        rospy.wait_for_service('yumi_teleop_host_ui_service')
        self.ui_service = str_str_service_wrapper(rospy.ServiceProxy('yumi_teleop_host_ui_service', str_str))

        #TODO: Subscribe to gripper events and republish
        self._l_gripper_sub = rospy.Subscriber('/dvrk/MTML/gripper_closed_event', Bool, self._gripper_callback_gen('left'))
        self._r_gripper_sub = rospy.Subscriber('/dvrk/MTMR/gripper_closed_event', Bool, self._gripper_callback_gen('right'))

    def _gripper_callback_gen(self, arm_name):
        def callback(gripper_closed):
            if self.cur_state == "teleop":
                self.ui_service("gripper", "('{0}',{1})".format(arm_name, gripper_closed.data))
        return callback

    def run(self):
        demo_names = eval(self.ui_service('list_demos'))
        self.menu_demo = tuple(demo_names) + ("Back",)

        "A FSM to interact with the teleop interface"
        self.ui.start()
        self.cur_menu = self.menu_main
        self.cur_state = "standby"
        while True:
            # gets current input
            ui_input = self.ui.list_view(self.cur_menu)

            # calls transition
            transition = getattr(self, "t_{0}".format(self.cur_state))
            self.cur_state, self.cur_menu, overlay = transition(ui_input)
            self.ui.set_overlay(overlay)

            if self.cur_state == "exit":
                break
        self.ui.stop()

    def t_standby(self, ui_input):
        if ui_input == "Collect Demos":
            return "demo_selection", self.menu_demo, True
        elif ui_input == "Sandbox":
            res = self.ui_service("teleop_start")
            return "teleop", self.menu_pause_teleop, False
        elif ui_input == "Quit":
            return "exit", None, None

    def t_demo_selection(self, ui_input):
        if ui_input == "Back":
            return "standby", self.menu_main, True
        res = self.ui_service("choose_demo", ui_input)
        return "teleop", self.menu_pause_teleop, False

    def t_teleop(self, ui_input):
        if ui_input == "Pause":
            res = self.ui_service("teleop_pause")
            return "teleop_paused", self.menu_resume_teleop, True
        elif ui_input == "Finish":
            res = self.ui_service("teleop_finish")
            return "standby", self.menu_main, True

    def t_teleop_paused(self, ui_input):
        if ui_input == "Resume":
            res = self.ui_service("teleop_resume")
            return "teleop", self.menu_pause_teleop, True
        elif ui_input == "Finish":
            res = self.ui_service("teleop_finish")
            return "standby", self.menu_main, True

if __name__ == "__main__":
    cid = 0
    ytc = YuMiTeleopClient(cid)
    ytc.run()
