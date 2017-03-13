#!/usr/bin/env python
"""
YuMiTeleopHost - core script that manages teleop and motion interfaces as well
as data recording for demonstrations.
Author: Jacky Liang
"""
from multiprocessing import Process, Queue, Manager
import argparse, os, sys, logging, rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from time import sleep, time
from Queue import Empty
import sys

from yumipy import YuMiRobot, YuMiSubscriber, YuMiState, YuMiControlException
from yumipy import YuMiConstants as ymc
from core import DataStreamRecorder, DataStreamSyncer, YamlConfig, RigidTransform
from perception import OpenCVCameraSensor, Kinect2PacketPipelineMode, Kinect2Sensor, PrimesenseSensor

from masters_control.srv import str_str, pose_str
from yumi_teleop import QueueEventsSub, TeleopExperimentLogger, T_to_ros_pose, ros_pose_to_T, IdentityFilter, DemoWrapper
from yumi_teleop.constants import MASTERS_GRIPPER_WIDTHS, GRIPPER_HOLD_WIDTH
import IPython

_L_SUB = "/yumi/l"
_R_SUB = "/yumi/r"

class _YuMiArmPoller(Process):

    def __init__(self, pose_q, cmds_q, ret_q, z, v, arm_name):
        Process.__init__(self)
        self.pose_q = pose_q
        self.cmds_q = cmds_q
        self.ret_q = ret_q

        self.z = z
        self.v = v
        self.arm_name = arm_name

        self.forward_poses = False
        self.filter = IdentityFilter()

    def run(self):
        self.move_counter = 0
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
                if not self.cmds_q.empty():
                    cmd = self.cmds_q.get()
                    if cmd[0] == 'forward':
                        self.forward_poses = cmd[1]
                        if cmd[1]:
                            self.filter.reset()
                            self.y.set_v(self.v)
                            self.y.set_z(self.z)
                            self.move_counter = 0
                    elif cmd[0] == 'stop':
                        break
                    elif cmd[0] == 'filter':
                        self.filter = cmd[1]
                    elif cmd[0] == 'count':
                        while self.ret_q.qsize() > 0:
                            self.ret_q.get_nowait()
                        self.ret_q.put(self.move_counter)
                    elif cmd[0] == 'method':
                        args = cmd[3]['args']
                        kwargs = cmd[3]['kwargs']
                        method_name = cmd[2]
                        if cmd[1] == 'both':
                            method = getattr(self.y, method_name)
                        elif cmd[1] == 'single':
                            method = getattr(self.arm, method_name)
                        retval = method(*args, **kwargs)
                        while self.ret_q.qsize() > 0:
                            self.ret_q.get_nowait()
                        if retval is not None:
                            self.ret_q.put(retval)
                elif self.forward_poses and not self.pose_q.empty():
                    self.move_counter += 1
                    try:
                        pose = self.pose_q.get()
                        filtered_pose = self.filter.apply(pose)
                        try:
                            res = self.arm.goto_pose(filtered_pose, relative=True)
                        except YuMiControlException:
                            logging.warn("Pose unreachable!")
                    except Empty:
                        pass
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
            },
            'ret': {
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
            'left': _YuMiArmPoller(self.qs['poses']['left'], self.qs['cmds']['left'], self.qs['ret']['left'], self.cfg['z'], self.cfg['v'], 'left'),
            'right': _YuMiArmPoller(self.qs['poses']['right'], self.qs['cmds']['right'], self.qs['ret']['right'], self.cfg['z'], self.cfg['v'], 'right'),
        }
        for poller in self.pollers.values():
            poller.start()
        self._call_both_poller('reset_home')
        self._call_both_poller('open_grippers')

        self._gripper_states = {
            'left': None,
            'right': None
        }

        self._recording_demo_name = None
        self._recording = False
        self._demos = {}
        self._cur_demo_start_time = None
        self._cur_demo_pause_time = None

        robot = {
            'both_poller': self._call_both_poller,
            'single_poller': self._call_single_poller,
            'left_ret_q': self.qs['ret']['left'],
            'right_ret_q': self.qs['ret']['right'],
        }

        for filename in os.listdir(self.cfg['demo_path']):
            if filename.endswith('demo.py'):
                full_filename = os.path.join(self.cfg['demo_path'], filename)
                demo_obj = DemoWrapper.load(full_filename, robot, self.set_filter)
                if demo_obj.name in self.cfg['display_demos']:
                    self._demos[demo_obj.name] = {
                        'filename': full_filename,
                        'obj': demo_obj
                    }

        # for debug
        self.debug_pub = rospy.Publisher('/yumi/host', String)

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
            try:
                self.webcam.stop()
            except Exception:
                pass
            try:
                self.primesense.stop()
            except Exception:
                pass
            while self.subs_q.qsize()>0:
                sub = self.subs_q.get()
                sub.stop()
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

        if self.cfg['data_srcs']['primesense']['use']:
            def ps_gen():
                ps_lst = []
                def ps_depth_frames():
                    if not ps_lst:
                        ps_lst.append(PrimesenseSensor())
                        ps_lst[0].start()
                    return ps_lst[0].frames()[0]
                return ps_depth_frames

            self.datas['primesense_depth'] = DataStreamRecorder('primesense_depth', ps_gen(), cache_path=cache_path, save_every=save_every)
            self.all_datas.append(self.datas['primesense_depth'])
            self.save_file_paths.append(self.cfg['data_srcs']['primesense']['T_path'])

        if self.cfg['data_srcs']['kinect']['use']:
            def kinect_gen():
                kinect = []
                def kinect_frames():
                    if not kinect:
                        kinect.append(Kinect2Sensor(device_num=self.cfg['data_srcs']['kinect']['n'],
                                                    packet_pipeline_mode=Kinect2PacketPipelineMode.OPENGL))
                        kinect[0].start()
                    return kinect[0].frames()[0]
                return kinect_frames

            self.datas['kinect_color'] = DataStreamRecorder('kinect_color', kinect_gen(), cache_path=cache_path, save_every=save_every)
            self.all_datas.append(self.datas['kinect_color'])
            self.save_file_paths.append(self.cfg['data_srcs']['kinect']['T_path'])


        self.subs_q = Queue()
        def yumi_sub_gen(include_left, include_right, name):
            subs = []
            def sub_data():
                arm = 'left' if include_left else 'right'
                if not subs:
                    subs.append(YuMiSubscriber(include_left=include_left, include_right=include_right,
                                                left_includes=(name,), right_includes=(name,)))
                    subs[0].start()
                    #self.subs_q.put(subs[0])
                return getattr(getattr(subs[0], arm), 'get_{}'.format(name)[:-1])()
            return sub_data

        for name in ('torques', 'poses', 'states'):
            self.datas[name] = {}
            for arm in ('left', 'right'):
                self.datas[name][arm] = DataStreamRecorder('{}_{}'.format(name, arm),
                                                            yumi_sub_gen(
                                                                arm == 'left',
                                                                arm == 'right',
                                                                name
                                                            ),
                                                            cache_path=cache_path,
                                                            save_every=save_every
                                                            )
        self.grippers_evs = {
            'left': QueueEventsSub(),
            'right': QueueEventsSub()
        }

        self.datas['grippers_evs'] = {
            'left': DataStreamRecorder('grippers_evs_left', self.grippers_evs['left'].get_event, cache_path=cache_path, save_every=save_every),
            'right': DataStreamRecorder('grippers_evs_right', self.grippers_evs['right'].get_event, cache_path=cache_path, save_every=save_every)
        }

        self.all_datas.extend([
            self.datas['grippers_evs']['left'],
            self.datas['grippers_evs']['right'],
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
            self._call_single_poller('left', 'get_pose')
            self._call_single_poller('right', 'get_pose')
            left_pose, right_pose = None, None
            while not isinstance(left_pose, RigidTransform):
                left_pose = self.qs['ret']['left'].get(block=True)
            while not isinstance(right_pose, RigidTransform):
                right_pose = self.qs['ret']['right'].get(block=True)

            left_ros_pose = T_to_ros_pose(left_pose)
            right_ros_pose = T_to_ros_pose(right_pose)

            self.init_pose_service(left=left_ros_pose, right=right_ros_pose)
            for q in self.qs['poses'].values():
                if q.qsize() > 0:
                    try:
                        q.get_nowait()
                    except Empty:
                        pass

    def _call_both_poller(self, method_name, *args, **kwargs):
        for poller in self.pollers.values():
            poller.send_cmd(('method', 'both', method_name, {'args':args, 'kwargs':kwargs}))

    def _call_single_poller(self, arm_name, method_name, *args, **kwargs):
        ret_q = self.qs['ret'][arm_name]
        while ret_q.qsize() > 0:
            ret_q.get_nowait()
        self.pollers[arm_name].send_cmd(('method', 'single', method_name, {'args':args, 'kwargs':kwargs}))

    def _teleop_begin(self, demo_name=None):
        self._recording = demo_name is not None
        if self._recording:
            self._recording_demo_name = demo_name
            self._demos[self._recording_demo_name]['obj'].setup()
        else: #sandbox mode
            self._call_single_poller('right', 'goto_state', ymc.AXIS_ALIGNED_STATES['inwards']['right'])
            self._call_single_poller('left', 'goto_state', ymc.AXIS_ALIGNED_STATES['inwards']['left'])
        rospy.loginfo("teleop in staging!")

    def _teleop_pause(self):
        if self.cur_state == "teleop_record":
            self.syncer.pause()
            self._cur_demo_pause_time = time()
        self._set_poller_forwards(False)

    def _teleop_resume(self):
        self._reset_masters_yumi_connector()
        if self.cur_state == "teleop_record_pause":
            self.syncer.resume()
            self._cur_demo_start_time += time() - self._cur_demo_pause_time
        self._set_poller_forwards(True)

    def _teleop_finish(self):
        self._set_poller_forwards(False)
        cur_time = time()
        if self._recording:
            self.syncer.pause()
            demo_time = cur_time - self._cur_demo_start_time
            self._demos[self._recording_demo_name]['obj'].takedown()
            self._recording = False
        self._call_both_poller('reset_home')
        self._call_both_poller('open_grippers')
        if self.cur_state == "teleop_record":
            while True:
                s = raw_input("Was the demo a success? [y/n] ")
                if s in ('y', 'n'):
                    break
                else:
                    print "Please only input 'y' or 'n'!\n"
            c = raw_input('Any comments? ')
            s = True if s == 'y' else False
            self.logger.save_demo_data(self._recording_demo_name,
                                        demo_time,
                                        s,
                                        self.cfg['supervisor'],
                                        self.save_file_paths + [self._demos[self._recording_demo_name]['filename']],
                                        self.all_datas,
                                        self.cfg['fps'],
                                        comments = c
                                        )
        else:
            dur = cur_time - self.sandbox_start_time
            sleep(0.5)
            for poller in self.pollers.values():
                poller.send_cmd(('count',))
            sleep(0.5)
            print 'getting ret'
            left_count = self.qs['ret']['left'].get(block=True)
            right_count = self.qs['ret']['right'].get(block=True)

            print 'duration: {}s'.format(dur)
            print 'counts left {} right {}'.format(left_count, right_count)
            print 'hz left {} right {}'.format(left_count/dur, right_count/dur)

    def t_teleop_staging(self, msg):
        if msg.req == 'teleop_production':
            self._reset_masters_yumi_connector()
            if self._recording:
                self.syncer.flush()
                self.syncer.resume(reset_time=True)
            self._set_poller_forwards(True)
            rospy.loginfo("beginning teleop!")
            if self._recording:
                self.cur_state = "teleop_record"
                self._cur_demo_start_time = time()
            else:
                self.cur_state = "teleop"
                self.sandbox_start_time = time()
        return "ok"

    def t_standby(self, msg):
        if msg.req == "teleop_start":
            self._teleop_begin()
            self.cur_state = 'teleop_staging'
        elif msg.req == "choose_demo":
            demo_name = msg.data
            self._teleop_begin(demo_name)
            self.cur_state = 'teleop_staging'
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

    def _get_gripper_state(self, s):
        if s > 0.9:
            return ('open',)
        if 0.4 < s <= 0.9:
            return ('hold',)
        # if 0.5 < s <= 0.6:
        #     return ('squeeze', 0.004, 7)
        # if 0.4 < s <= 0.6:
        #     return ('squeeze', 0.003, 9)
        # if 0.3 < s <= 0.4:
        #     return ('squeeze', 0.002, 11)
        # if 0.2 < s <= 0.4:
        #     return ('squeeze', 0.001, 13)
        else:
            return ('close',)

    def cmd_gripper(self, data):
        cmd = eval(data)
        arm_name = cmd[0]
        if cmd[1] == 'binary':
            if cmd[2]:
                self._call_single_poller(arm_name, 'close_gripper')
                self.grippers_evs[arm_name].put_event('close_gripper')
            else:
                self._call_single_poller(arm_name, 'open_gripper')
                self.grippers_evs[arm_name].put_event('open_gripper')
        elif cmd[1] == 'continuous':
            scale = (cmd[2] - MASTERS_GRIPPER_WIDTHS[arm_name]['min']) / (MASTERS_GRIPPER_WIDTHS[arm_name]['max'] - MASTERS_GRIPPER_WIDTHS[arm_name]['min'])
            scale = max(min(scale, 1), 0)
            self.debug_pub.publish('scale {}: {}'.format(arm_name, scale))

            if self._gripper_states[arm_name] is None:
                self._gripper_states[arm_name] = self._get_gripper_state(scale)
                return

            cur_state = self._get_gripper_state(scale)
            last_state = self._gripper_states[arm_name]
            if cur_state != last_state:
                self.debug_pub.publish('state change {} -> {}'.format(last_state, cur_state))
                cur_name = cur_state[0]
                last_name = last_state[0]
                if cur_name == 'open':
                    self._call_single_poller(arm_name, 'open_gripper')
                    self.grippers_evs[arm_name].put_event('open_gripper')
                elif cur_name == 'hold':
                    self._call_single_poller(arm_name, 'move_gripper', GRIPPER_HOLD_WIDTH, wait_for_res=False)
                    self.grippers_evs[arm_name].put_event(['move_gripper', GRIPPER_HOLD_WIDTH])
                elif cur_name == 'squeeze' and last_name == 'close':
                    width = cur_state[1]
                    self._call_single_poller(arm_name, 'move_gripper', width, wait_for_res=False)
                    self.grippers_evs[arm_name].put_event(['move_gripper', width])
                elif cur_name == 'squeeze' and last_name == 'hold':
                    force = cur_state[2]
                    self._call_single_poller(arm_name, 'close_gripper', force, wait_for_res=False)
                    self.grippers_evs[arm_name].put_event(['close_gripper', force])
                elif cur_name == 'squeeze' and last_name == 'squeeze':
                    width, force = cur_state[1:]
                    last_width = last_state[1]
                    if width < last_width:
                        self._call_single_poller(arm_name, 'close_gripper', force, wait_for_res=False)
                        self.grippers_evs[arm_name].put_event(['close_gripper', force])
                    else:
                        self._call_single_poller(arm_name, 'move_gripper', width, wait_for_res=False)
                        self.grippers_evs[arm_name].put_event(['move_gripper', width])
                elif cur_name == 'close':
                    self._call_single_poller(arm_name, 'close_gripper')
                    self.grippers_evs[arm_name].put_event('close_gripper')
                self._gripper_states[arm_name] = cur_state


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.INFO)
    parser = argparse.ArgumentParser(description='YuMi Teleop Host')
    parser.add_argument('-c', '--config_path', type=str, default='cfg/demo_config.yaml', help='path to config file')
    args = parser.parse_args()

    yth = YuMiTeleopHost(args.config_path)
    yth.run()
