#!/usr/bin/env python
"""
Script to be ran on the 'client' machine on a masters-YuMi teleop setup. Client
machine is the computer connected to the masters' vision system.
Author: Jacky Liang
"""
import rospy, cv2, argparse
import numpy as np
from multiprocessing import Process, Queue
from cv_bridge import CvBridge
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image
from masters_control.srv import str_str
from core import YamlConfig
from time import sleep

from yumi_teleop import str_str_service_wrapper
import IPython

class UI(Process):

    BG_COLOR = (0,0,255)
    TXT_COLOR = (0,0,0)
    HI_COLOR = (0,255,255)

    def __init__(self, cfg):
        Process.__init__(self)
        self.req_q = Queue()
        self.res_q = Queue()
        self.cfg = cfg
        self.debug = self.cfg['debug']
        self.frame_cam_map = {
              'left':{
                  'cur': 0,
                  'cams': ['left', 'side'],
                },
              'right':{
                  'cur': 0,
                  'cams': ['right']
                }
            }
        self.cams = {}
        self.subs = []
        self.cv_bridge = CvBridge()

        if self.debug:
            self._black_frame = np.dstack([np.zeros((480,640))*0.1]*3)
        else:
            for name in self.cfg['cams']:
                self.cams[name] = self.gen_camera_frames(name, self.cfg['cams'][name]['topic_name'])

    def gen_camera_frames(self, view_name, topic_name):
        rospy.loginfo("gen for {}".format(topic_name))
        frames_q = Queue(maxsize=1)
        def enqueue(msg):
            while frames_q.qsize() > 0:
                frames_q.get_nowait()
            image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            frames_q.put(image[...,::-1])
        self.subs.append(rospy.Subscriber(topic_name, Image, enqueue))

        def frames():
            return frames_q.get()
        return frames

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
        self.left = cv2.namedWindow("left", cv2.cv.CV_WINDOW_NORMAL)
        self.right = cv2.namedWindow("right", cv2.cv.CV_WINDOW_NORMAL)
        self.debug_left = cv2.namedWindow("debug_left", cv2.cv.CV_WINDOW_NORMAL)
        self.debug_right = cv2.namedWindow("debug_right", cv2.cv.CV_WINDOW_NORMAL)

        self.list_view = []
        self.list_index = 0
        self.show_overlay = True

        while True:
            if self.debug:
                frame1 = self._black_frame.copy()
                frame2 = self._black_frame.copy()
            else:
                frame1_name = self.frame_cam_map['left']['cams'][self.frame_cam_map['left']['cur']]
                frame2_name = self.frame_cam_map['right']['cams'][self.frame_cam_map['right']['cur']]
                frame1 = self.cams[frame1_name]()
                frame2 = self.cams[frame2_name]()
                if self.cfg['cams'][frame1_name]['flip']:
                    frame1 = np.rot90(np.rot90(frame1)).copy()
                if self.cfg['cams'][frame2_name]['flip']:
                    frame2 = np.rot90(np.rot90(frame2)).copy()

            pedals_io = {'down':False, 'overlay':False, 'select':False}

            if not self.req_q.empty():
                req = self.req_q.get()
                if req[0] == "stop":
                    break
                elif req[0] == "list_view":
                    self.list_view = req[1]
                    self.list_index = 0
                elif req[0] == "overlay":
                    self.show_overlay = req[1]
                elif req[0] == "pedals":
                    pedals_io = req[1]

            if self.show_overlay and len(self.list_view) > 0:
                overlay1 = self.gen_list_view(frame1)
                overlay2 = self.gen_list_view(frame2)
                cv2.addWeighted(overlay1, 0.7, frame1, 0.3, 0, frame1)
                cv2.addWeighted(overlay2, 0.7, frame2, 0.3, 0, frame2)

            cv2.imshow('left', frame1)
            cv2.imshow('right', frame2)
            cv2.imshow('debug_left', frame1)
            cv2.imshow('debug_right', frame2)

            pressed = cv2.waitKey(1) & 0xFF
            if self.show_overlay:
                if pressed == ord('w'):
                    self.list_index -= 1
                elif pressed == ord('s') or pedals_io['down']:
                    self.list_index += 1
                elif pressed == ord('d') or pedals_io['select']:
                    self.res_q.put(self.list_view[self.list_index])
                    self.list_view = []
                if len(self.list_view) > 0:
                    self.list_index = self.list_index % len(self.list_view)
            else:
                if pressed == ord('s') or pedals_io['down']: # cycle right
                    self.frame_cam_map['right']['cur'] = (self.frame_cam_map['right']['cur'] + 1) % len(self.frame_cam_map['right']['cams'])
                elif pressed == ord('d') or pedals_io['select']: # cycle left cam
                    self.frame_cam_map['left']['cur'] = (self.frame_cam_map['left']['cur'] + 1) % len(self.frame_cam_map['left']['cams'])
            if pressed == ord('a') or pedals_io['overlay']:
                self.show_overlay = not self.show_overlay

        cv2.destroyAllWindows()

    def list_view(self, lst):
        self.req_q.put(("list_view", lst))
        while self.res_q.empty():
            pass
        return self.res_q.get()

    def set_overlay(self, overlay):
        print 'setting overlay to ', overlay
        self.req_q.put(("overlay", overlay))

    def stop(self):
        self.req_q.put(("stop",))
        for sub in self.subs:
            sub.unregister()

    def set_pedals(self, pedals_io):
        self.req_q.put(("pedals", pedals_io))

class YuMiTeleopClient:

    def __init__(self, cfg):
        self.cfg = cfg
        rospy.init_node("yumi_teleop_client")
        rospy.loginfo("Init YuMiTeleopClient")
        rospy.loginfo("Waiting for host ui service...")
        rospy.wait_for_service('yumi_teleop_host_ui_service')
        self.ui_service = str_str_service_wrapper(rospy.ServiceProxy('yumi_teleop_host_ui_service', str_str))
        self.teleop_confirmation_service = str_str_service_wrapper(rospy.ServiceProxy('yumi_teleop_confirmation_service', str_str))
        rospy.loginfo("UI Service established!")

        self.menu_main = ("Collect Demos", "Sandbox", "Quit")
        self.menu_pause_teleop = ("Pause", "Finish")
        self.menu_resume_teleop = ("Resume", "Finish")
        self.menu_teleop_staging = ("Go!",)
        self.menu_demo = None

        self.ui = UI(self.cfg)

        self._select_sub = rospy.Subscriber('/dvrk/footpedals/camera', Bool, self._pedals_call_back_gen('select'))
        self._overlay_sub = rospy.Subscriber('/dvrk/footpedals/camera_plus', Bool, self._pedals_call_back_gen('overlay'))
        self._down_sub = rospy.Subscriber('/dvrk/footpedals/camera_minus', Bool, self._pedals_call_back_gen('down'))
        self._clutch_sub = rospy.Subscriber('/dvrk/footpedals/clutch', Bool, self._clutch_callback)
        self._clutch_down = False
        self.cur_state = None

        self.last_gripper_widths = {
              'right': None,
              'left': None
            }

        if self.cfg['grippers'] == 'binary':
            self._l_gripper_sub = rospy.Subscriber('/dvrk/MTML/gripper_closed_event', Bool, self._gripper_callback_gen('left'))
            self._r_gripper_sub = rospy.Subscriber('/dvrk/MTMR/gripper_closed_event', Bool, self._gripper_callback_gen('right'))
        elif self.cfg['grippers'] == 'continuous':
            self._l_gripper_sub = rospy.Subscriber('/dvrk/MTML/gripper_position_current', Float32, self._gripper_callback_gen('left'))
            self._r_gripper_sub = rospy.Subscriber('/dvrk/MTMR/gripper_position_current', Float32, self._gripper_callback_gen('right'))
        else:
            raise ValueError("Unknown gripper mode! Can only be binary or continuous, got {}".format(self.cfg['grippers']))

        rospy.on_shutdown(self._shutdown_hook_gen())

    def _shutdown_hook_gen(self):
        def shutdown_hook():
            self.ui.stop()
            self._l_gripper_sub.unregister()
            self._r_gripper_sub.unregister()
            self._select_sub.unregister()
            self._overlay_sub.unregister()
            self._down_sub.unregister()
            self._clutch_sub.unregister()
        return shutdown_hook

    def _pedals_call_back_gen(self, pedal):
        def callback(msg):
            is_down = msg.data
            if is_down:
                pedals_io = {'overlay':False, 'down':False, 'select':False}
                pedals_io[pedal] = True
                self.ui.set_pedals(pedals_io)
        return callback

    def _gripper_callback_gen(self, arm_name):
        def callback(gripper_ev):
            if self.cur_state == "teleop" and not self._clutch_down:

                if self.cfg['grippers'] == 'continuous':
                    if self.last_gripper_widths[arm_name] is None:
                        self.last_gripper_widths[arm_name] = gripper_ev.data
                    if abs(gripper_ev.data - self.last_gripper_widths[arm_name]) < 0.1:
                        return

                self.ui_service("gripper", "('{0}','{1}', {2})".format(arm_name, self.cfg['grippers'], gripper_ev.data))
        return callback

    def _clutch_callback(self, msg):
        self._clutch_down = msg.data

    def run(self):
        demo_names = eval(self.ui_service('list_demos'))
        self.menu_demo = tuple(demo_names) + ("Back",)

        # A FSM to interact with the teleop interface
        self.ui.start()
        self.cur_menu = self.menu_main
        self.cur_state = "standby"
        while True:
            # gets current input
            ui_input = self.ui.list_view(self.cur_menu)

            # calls transition
            transition = getattr(self, "t_{0}".format(self.cur_state))
            print "calling transtion on t_{}".format(self.cur_state)

            self.cur_state, self.cur_menu, overlay = transition(ui_input)
            self.ui.set_overlay(overlay)

            if self.cur_state == "exit":
                break
        self.ui.stop()

    def t_teleop_staging(self, ui_input):
        if ui_input == "Go!":
            _ = self.ui_service("teleop_production",)
            return "teleop", self.menu_pause_teleop, False

    def t_standby(self, ui_input):
        if ui_input == "Collect Demos":
            return "demo_selection", self.menu_demo, True
        elif ui_input == "Sandbox":
            _ = self.ui_service("teleop_start",)
            return "teleop_staging", self.menu_teleop_staging, True
        elif ui_input == "Quit":
            return "exit", None, None

    def t_demo_selection(self, ui_input):
        if ui_input == "Back":
            return "standby", self.menu_main, True
        _ = self.ui_service("choose_demo", ui_input)
        return "teleop_staging", self.menu_teleop_staging, True

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
    parser = argparse.ArgumentParser(description='YuMi Teleop Client')
    parser.add_argument('-c', '--config_path', type=str, default='cfg/client_config.yaml', help='path to config file')
    args = parser.parse_args()

    cfg = YamlConfig(args.config_path)
    ytc = YuMiTeleopClient(cfg)
    ytc.run()
