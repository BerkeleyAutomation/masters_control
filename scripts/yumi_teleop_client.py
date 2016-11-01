"""
Script to be ran on the 'client' machine on a masters-YuMi teleop setup. Client
machine is the computer connected to the masters' vision system.
Author: Jacky Liang
"""
#import rospy
import cv2
import numpy as np
from multiprocess import Process, Queue

class _UI(Process):

    BG_COLOR = (0,0,255)
    TXT_COLOR = (0,0,0)
    HI_COLOR = (0,255,255)

    def __init__(self, req_q, res_q):
        Process.__init__(self)
        self.cam = cv2.VideoCapture(0)
        self.left = cv2.namedWindow("left")
        self.right = cv2.namedWindow("right")

        self.req_q = req_q
        self.res_q = res_q

        self.list_view = []
        self.list_index = 0
        self.show_overlay = True

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

            rec_color = _UI.HI_COLOR if i == self.list_index else _UI.BG_COLOR

            cv2.rectangle(overlay, top_left, bottom_right, rec_color,-1)
            cv2.putText(overlay, item, text_left, cv2.FONT_HERSHEY_SIMPLEX, 0.7, _UI.TXT_COLOR, 1)

        cv2.putText(overlay, "AUTOLab YuMi Teleop Interface", (20, overlay.shape[0]-10), cv2.FONT_HERSHEY_PLAIN, 1.0, (255,255,255), 1)

        return overlay

    def run(self):
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
                self.list_index = np.clip(self.list_index, 0, len(self.list_view))
            if pressed == ord('a'):
                self.show_overlay = not self.show_overlay

        self.stop()

    def stop(self):
        self.cam.release()
        cv2.destroyAllWindows()

class UI:

    def __init__(self):
        self._req_q = Queue()
        self._res_q = Queue()
        self._ui = _UI(self._req_q, self._res_q)
        self._ui.start()

    def list_view(self, lst):
        self._req_q.put(("list_view", lst))
        while self._res_q.empty():
            pass
        return self._res_q.get()

    def set_overlay(self, overlay):
        self._req_q.put(("overlay", overlay))

    def stop(self):
        self._req_q.put(("stop"))

class YuMiTeleopClient:

    def __init__(self):
        self.menu_main = ("Collect Demos", "Sandbox", "Quit")

        #TODO: Load actual demo names by requestin host service
        self.menu_demo = ("demo1", "demo2", "Back")
        self.menu_pause_teleop = ("Pause", "Finish")
        self.menu_resume_teleop = ("Resume", "Finish")
        self.ui = _UI(self.menu_main)

        """
        States:
            standby
            demo_selection
            teleop
            teleop_paused
        """

    def run(self):
        "A FSM to interact with the teleop interface"
        self.ui.run()
        cur_menu = self.menu_main
        cur_state = "standby"
        while True:
            # gets current input
            ui_input = self.ui.list_view(cur_menu)

            # calls transition
            transition = getattr(self, "t_{0}".format(cur_state))
            cur_state, cur_menu, overlay = transition(ui_input)
            self.ui.set_overlay(overlay)

            if cur_state == "exit":
                break
        self.ui.stop()

    def t_standby(self, ui_input):
        if ui_input == "Collect Demos":
            return "demo_selection", self.menu_demo, True
        elif ui_input == "Sandbox":
            #TODO: signal to host
            return "teleop", self.menu_pause_teleop, False
        elif ui_input == "Quit":
            return "exit", None, None

    def t_demo_selection(self, ui_input):
        if ui_input == "Back":
            return "standby", self.menu_main, True
        #TODO: signal to host w/ the selection as ui_input
        return "teleop", self.menu_pause_teleop, False

    def t_teleop(self, ui_input):
        if ui_input == "Pause":
            #TODO: signal to host for teleop pause
            return "teleop_paused", self.menu_resume_teleop, True
        elif ui_input == "Finish":
            #TODO: signal to host for teleop finish
            return "standby", self.menu_main, True

    def t_telop_paused(self, ui_input):
        if ui_input == "Resume":
            #TODO: signal to host for teleop resume
            return "teleop", self.menu_pause_teleop, True
        elif ui_input == "Finish":
            #TODO: signal to host for teleop Finish
            return "standby", self.menu_main, True

if __name__ == "__main__":
    ytc = YuMiTeleopClient()
    ytc.run()
