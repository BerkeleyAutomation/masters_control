#!/usr/bin/env python
"""
YuMiTeleopHost - core script that manages teleop and motion interfaces as well
as data recording for demonstrations.
Author: Jacky Liang
"""
from multiprocessing import Process, Queue
import argparse, os, sys, logging, rospy
from geometry_msgs.msg import Pose
from time import sleep
from Queue import Empty

from yumipy import YuMiRobot, YuMiSubscriber, YuMiState
from yumipy import YuMiConstants as ymc
from core import DataStreamRecorder, DataStreamSyncer, YamlConfig
from perception import OpenCVCameraSensor, Kinect2PacketPipelineMode, Kinect2Sensor

from masters_control.srv import str_str, pose_str
from yumi_teleop import QueueEventsSub, TeleopExperimentLogger, T_to_ros_pose, ros_pose_to_T, IdentityFilter, DemoWrapper

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
        self.filter = IdentityFilter()

    def run(self):
        logging.getLogger().setLevel(ymc.LOGGING_LEVEL)
        if self.arm_name == "left":
            self.y = YuMiRobot(include_right=False)
            self.arm = self.y.left
        elif self.arm_name == "right":
            self.y = YuMiRobot(include_left=False)
            self.arm = self.y.right
        self.y.set_v(self.v)
        self.y.set_z(self.z)

        while True:
            try:
                if self.forward_poses and not self.pose_q.empty():
                    try:
                        pose = self.pose_q.get()
                        filtered_pose = self.filter.apply(pose)
                        try:
                            res = self.arm.goto_pose(filtered_pose, relative=True)
                        except YuMiControlException:
                            pass
                    except Empty:
                        pass
                if not self.cmds_q.empty():
                    cmd = self.cmds_q.get()
                    if cmd[0] == 'forward':
                        self.forward_poses = cmd[1]
                        if cmd[1]:
                            self.filter.reset()
                            self.y.set_v(self.v)
                            self.y.set_z(self.z)
                    elif cmd[0] == 'stop':
                        break
                    elif cmd[0] == 'filter':
                        self.filter = cmd[1]
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
            except KeyboardInterrupt:
                logging.debug("Shutting down {0} arm poller".format(self.arm_name))

        self.y.stop()

    def stop(self):
        self.cmds_q.put(('stop',))

    def set_forward(self, val):
        self.cmds_q.put(('forward', val))

    def send_cmd(self, packet):
        self.cmds_q.put(packet)

    def set_filter(self, motion_filter):
        self.cmds_q.put(('filter', motion_filter))

class YuMiTeleopHost:

    def __init__(self, cfg_path):
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

        self.cfg_path = cfg_path
        self.cfg = YamlConfig(self.cfg_path)

        self.save_file_paths = [self.cfg_path]

        if self.cfg['record']:
            self.logger = TeleopExperimentLogger(self.cfg['output_path'], self.cfg['supervisor'])

        self.pollers = {
            'left': _YuMiArmPoller(self.qs['poses']['left'], self.qs['cmds']['left'], self.cfg['z'], self.cfg['v'], 'left'),
            'right': _YuMiArmPoller(self.qs['poses']['right'], self.qs['cmds']['right'], self.cfg['z'], self.cfg['v'], 'right'),
        }
        for poller in self.pollers.values():
            poller.start()
        self._call_both_poller('reset_home')
        self._call_both_poller('open_grippers')

        self.ysub = YuMiSubscriber()
        self.ysub.start()

        self._recording_demo_name = None
        self._recording = False
        self._demos = {}

        robot = {
            'both_poller': self._call_both_poller,
            'single_poller': self._call_single_poller,
            'sub': self.ysub
        }

        for filename in os.listdir(self.cfg['demo_path']):
            if filename.endswith('demo.py'):
                full_filename = os.path.join(self.cfg['demo_path'], filename)
                demo_obj = DemoWrapper.load(full_filename, robot, self.set_filter)
                self._demos[demo_obj.name] = {
                    'filename': full_filename,
                    'obj': demo_obj
                }

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
            try:
                self.webcam.stop()
            except Exception:
                pass
            self.syncer.stop()

        return shutdown_hook

    def set_filter(self, motion_filter):
        for poller in self.pollers.values():
            poller.set_filter(motion_filter)

    def dispatcher(self, msg):
        rospy.loginfo("Rcv\n{0}\n".format(msg))
        transition = getattr(self, "t_{0}".format(self.cur_state))
        res = transition(msg)
        rospy.loginfo("Snt {0}".format(res))
        return res

    def run(self):
        rospy.init_node("yumi_teleop_host")
        rospy.loginfo("Init YuMiTeleopHost")

        # establishing data recording
        rospy.loginfo("Setting up data streams...")
        self.datas = {}
        self.all_datas = []

        cache_path, save_every = self.cfg['cache_path'], self.cfg['save_every']

        if self.cfg['data_srcs']['webcam']['use']:
            self.webcam = OpenCVCameraSensor(self.cfg['data_srcs']['webcam']['n'])
            self.webcam.start()
            self.datas['webcam'] = DataStreamRecorder('webcam', self.webcam.frames, cache_path=cache_path, save_every=save_every)
            self.all_datas.append(self.datas['webcam'])
            self.save_file_paths.append(self.cfg['data_srcs']['webcam']['T_path'])

        if self.cfg['data_srcs']['kinect']['use']:
            def kinect_gen():
                kinect = []
                def kinect_frames():
                    if not kinect:
                        kinect.append(Kinect2Sensor(device_num=self.cfg['data_srcs']['kinect']['n'],
                                                    packet_pipeline_mode=Kinect2PacketPipelineMode.OPENGL))
                        kinect[0].start()
                    return kinect[0].frames()[:2]
                return kinect_frames

            self.datas['kinect'] = DataStreamRecorder('kinect', kinect_gen(), cache_path=cache_path, save_every=save_every)
            self.all_datas.append(self.datas['kinect'])
            self.save_file_paths.append(self.cfg['data_srcs']['kinect']['T_path'])

        self.datas['poses'] = {
            'left': DataStreamRecorder('poses_left', self.ysub.left.get_pose, cache_path=cache_path, save_every=save_every),
            'right': DataStreamRecorder('poses_right', self.ysub.right.get_pose, cache_path=cache_path, save_every=save_every)
        }
        self.datas['states'] = {
            'left': DataStreamRecorder('states_left', self.ysub.left.get_state, cache_path=cache_path, save_every=save_every),
            'right': DataStreamRecorder('states_right', self.ysub.right.get_state, cache_path=cache_path, save_every=save_every)
        }
        self.datas['torques'] = {
            'left': DataStreamRecorder('torques_left', self.ysub.left.get_torque, cache_path=cache_path, save_every=save_every),
            'right': DataStreamRecorder('torques_right', self.ysub.right.get_torque, cache_path=cache_path, save_every=save_every)
        }

        self.grippers_bool = {
            'left': QueueEventsSub(),
            'right': QueueEventsSub()
        }

        self.datas['grippers_bool'] = {
            'left': DataStreamRecorder('grippers_bool_left', self.grippers_bool['left'].get_event, cache_path=cache_path, save_every=save_every),
            'right': DataStreamRecorder('grippers_bool_right', self.grippers_bool['right'].get_event, cache_path=cache_path, save_every=save_every)
        }

        self.all_datas.extend([
            self.datas['grippers_bool']['left'],
            self.datas['grippers_bool']['right'],
            self.datas['poses']['left'],
            self.datas['poses']['right'],
            self.datas['states']['left'],
            self.datas['states']['right'],
            self.datas['torques']['left'],
            self.datas['torques']['right']
        ])

        self.syncer = DataStreamSyncer(self.all_datas, self.cfg['fps'])
        self.syncer.start()
        rospy.loginfo("Waiting for initial flush...")
        sleep(3)
        self.syncer.pause()
        self.syncer.flush()
        rospy.loginfo("Done!")

        rospy.loginfo("Setting up motion subscribers...")
        self.subs = {
            'left': rospy.Subscriber(_L_SUB, Pose, self._enqueue_pose_gen(self.qs['poses']['left'])),
            'right': rospy.Subscriber(_R_SUB, Pose, self._enqueue_pose_gen(self.qs['poses']['right'])),
        }
        rospy.loginfo("Done!")

        rospy.on_shutdown(self._shutdown_hook_gen())
        self.cur_state = 'standby'

        if not self.cfg['debug']:
            rospy.loginfo("Waiting for Teleop Pose Service...")
            rospy.wait_for_service('masters_yumi_transform_reset_init_poses')
            self.init_pose_service = rospy.ServiceProxy('masters_yumi_transform_reset_init_poses', pose_str)
            rospy.loginfo("Established Teleop Pose Service!")
        else:
            rospy.loginfo("In debug mode! Will not wait for masters yumi connector.")

        self.ui_service = rospy.Service('yumi_teleop_host_ui_service', str_str, self.dispatcher)
        rospy.loginfo("All setup done! Serving UI Service...")

        rospy.spin()

    def _set_poller_forwards(self, val):
        for poller in self.pollers.values():
            poller.set_forward(val)

    def _reset_masters_yumi_connector(self):
        if not self.cfg['debug']:
            left_pose = T_to_ros_pose(self.ysub.left.get_pose(timestamp=False))
            right_pose = T_to_ros_pose(self.ysub.right.get_pose(timestamp=False))

            self.init_pose_service(left=left_pose, right=right_pose)

    def _call_both_poller(self, method_name, *args, **kwargs):
        for poller in self.pollers.values():
            poller.send_cmd(('method', 'both', method_name, {'args':args, 'kwargs':kwargs}))

    def _call_single_poller(self, arm_name, method_name, *args, **kwargs):
        self.pollers[arm_name].send_cmd(('method', 'single', method_name, {'args':args, 'kwargs':kwargs}))

    def _teleop_begin(self, demo_name=None):
        self._recording = demo_name is not None
        if self._recording:
            self._recording_demo_name = demo_name
            self._demos[self._recording_demo_name]['obj'].setup()
        else:
            self._call_single_poller('right', 'goto_state', YuMiState([36.42, -117.3, 35.59, 50.42, 46.19, 66.02, -100.28]))
            self._call_single_poller('left', 'goto_state', YuMiState([-36.42, -117.3, 35.59, -50.42, 46.19, 113.98, 100.28]))

        sleep(3)
        rospy.loginfo("beginning teleop!")
        self._reset_masters_yumi_connector()

        if self._recording:
            self.syncer.resume(reset_time=True)
        self._set_poller_forwards(True)

    def _teleop_pause(self):
        if self.cur_state == "teleop_record":
            self.syncer.pause()
        self._set_poller_forwards(False)

    def _teleop_resume(self):
        self._reset_masters_yumi_connector()
        if self.cur_state == "teleop_record_pause":
            self.syncer.resume()
        self._set_poller_forwards(True)

    def _teleop_finish(self):
        self._set_poller_forwards(False)
        if self._recording:
            self._demos[self._recording_demo_name]['obj'].takedown()
            self._recording = False
        self._call_both_poller('reset_home')
        self._call_both_poller('open_grippers')
        if self.cur_state == "teleop_record":
            self.syncer.pause()
            self.logger.save_demo_data(self._recording_demo_name,
                                        self.cfg['supervisor'],
                                        self.save_file_paths + [self._demos[self._recording_demo_name]['filename']],
                                        self.all_datas,
                                        self.cfg['fps']
                                        )

    def t_standby(self, msg):
        if msg.req == "teleop_start":
            self._teleop_begin()
            self.cur_state = 'teleop'
        elif msg.req == "choose_demo":
            demo_name = msg.data
            self._teleop_begin(demo_name)
            self.cur_state = 'teleop_record'
        elif msg.req == "list_demos":
            res = repr(self._demos.keys())
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
            self.grippers_bool[arm_name].put_event('close_gripper')
        else:
            self._call_single_poller(arm_name, 'open_gripper')
            self.grippers_bool[arm_name].put_event('open_gripper')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='YuMi Teleop Host')
    parser.add_argument('-c', '--config_path', type=str, default='cfg/demo_config.yaml', help='path to config file')
    args = parser.parse_args()

    yth = YuMiTeleopHost(args.config_path)
    yth.run()
