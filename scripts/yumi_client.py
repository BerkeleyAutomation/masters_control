#!/usr/bin/env python
'''
Script to command YuMi by listening to dvrk masters
Author: Jacky
'''
import logging
import sys
import os
import rospy
import numpy as np
import argparse
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from time import sleep, time

from alan.control import YuMiConstants as YMC
from alan.control import YuMiRobot, YuMiState
from alan.core import RigidTransform

import IPython

from multiprocessing import Process, Queue

class _YuMiArmPoller(Process):

    def __init__(self, arm, pose_q, times_q):
        Process.__init__(self)
        self.pose_q = pose_q
        self.arm = arm
        self.times_q = times_q

    def run(self):
        i =0 
        while True:
            if not self.pose_q.empty():
                pose = self.pose_q.get()

                if pose is None:
                    break

                start_time = time()
                res = self.arm.goto_pose(pose)
                end_time = time()
                total_time = end_time - start_time
                self.times_q.put({
                            'total': total_time,
                            'motion': res['time'],
                            'x': res['pose'].translation[0]
                        })

class _RateLimiter:

    def __init__(self, period):
        self.period = period
        self.last_time = None

    @property
    def ok(self):
        if self.last_time is None:
            self.last_time = time()
            return True

        cur_time = time()
        if cur_time - self.last_time >= self.period:
            self.last_time = time()
            return True
        return False

class YuMiClient:

    _L_REL = '/MTML_YuMi/position_cartesian_current_rel'
    _R_REL = '/MTMR_YuMi/position_cartesian_current_rel'
    _L_GRIPPER_CLOSE = '/dvrk/MTML/gripper_closed_event'
    _R_GRIPPER_CLOSE = '/dvrk/MTMR/gripper_closed_event'

    _T_MC_YCR = RigidTransform(rotation=[[0,-1,0],
                                        [1,0,0],
                                        [0,0,1]],
                                from_frame='yumi_current_ref', to_frame='masters_current')
    
    _T_YIR_MI = RigidTransform(rotation=[[0,1,0],
                                          [-1,0,0],
                                          [0,0,1]],
                                from_frame='masters_init', to_frame='yumi_init_ref')

    _MASTERS_TO_YUMI_SCALE = 1
    _POSE_DIFF_THRESHOLD = 0.001 # this is in m

    #TODO: Test and set this value
    _ROT_MAG_SCALE = 0

    def __init__(self, output_path, z):
        self.output_path = output_path
        rospy.init_node('yumi_client', anonymous=True)
        self.yumi = YuMiRobot(log_pose_histories=True, include_right=False)
        self.yumi.set_z(z)
        self.yumi.set_v(200)
        self.yumi.reset_home()
        #self.yumi.left.goto_state(YuMiState([-76.64, -116.38, 14.2, 60.81, 85.89, 26.58, 51]))
        #self.yumi.right.goto_state(YuMiState([60.39, -101.29, -18.86, -34.97, 104.81, -39.85, -46.82]))
        self.yumi.set_v(1500)

        self._pose_q = {
            'left': Queue(maxsize=1),
            #'right': Queue(maxsize=1)
        }

        self._times_q = {
            'left': Queue(),
            #'right': Queue()
        }

        self._yumi_poller = {
            'left': _YuMiArmPoller(self.yumi.left, self._pose_q['left'], self._times_q['left']),
            #'right': _YuMiArmPoller(self.yumi.right, self._pose_q['right'], self._times_q['right'])
        }

        self._yumi_poller['left'].start()
        #self._yumi_poller['right'].start()

        sleep(1)
        self.T_w_yi = {
            'left': self.yumi.left.get_pose().as_frames('yumi_init', 'world'),
            #'right': self.yumi.right.get_pose().as_frames('yumi_init', 'world')
        }
        self.T_yi_yir = {
            'left': RigidTransform(rotation=self.T_w_yi['left'].inverse().rotation, 
                                    from_frame='yumi_init_ref', to_frame='yumi_init'),
            #'right': RigidTransform(rotation=self.T_w_yi['right'].inverse().rotation, 
                                    #from_frame='yumi_init_ref', to_frame='yumi_init')
        }
        self.T_ycr_yc = {
            'left': RigidTransform(rotation=self.T_w_yi['left'].rotation, 
                                    from_frame='yumi_current', to_frame='yumi_current_ref'),
            #'right': RigidTransform(rotation=self.T_w_yi['right'].rotation, 
                              #      from_frame='yumi_current', to_frame='yumi_current_ref')
        }
        self.last_T_w_yc = {
            'left': self.T_w_yi['left'].copy(),
         #   'right': self.T_w_yi['right'].copy()
        }
        self.rate_limiter = {
            'left': _RateLimiter(YMC.COMM_PERIOD),
          #  'right': _RateLimiter(YMC.COMM_PERIOD)
        }

    def _shutdown_hook_gen(self):
        def shutdown_hook():
            rospy.loginfo("Shutting down yumi client..")
            for q in self._pose_q.values():
                q.put(None)
            self.yumi.stop()
            self._l_gripper_sub.unregister()
            #self._r_gripper_sub.unregister()
            #self._r_motion_sub.unregister()
            self._l_motion_sub.unregister()

            rospy.loginfo("Saving motion data...")
            self.save_data('left')
            #self.save_data('right')
            rospy.loginfo("Done!")
        return shutdown_hook

    def save_data(self, arm_name):
        times_q = self._times_q[arm_name]
        total_times = []
        motion_times = []
        xs = [0]
        while not times_q.empty():
            time_data = times_q.get()
            total_times.append(time_data['total'])
            motion_times.append(time_data['motion'])
            xs.append(time_data['x'])

        t = np.array(total_times)
        m = np.array(motion_times)
        d = t - m
        
        if len(d) == 0:
            return

        output_path = os.path.join(self.output_path, arm_name)
        if not os.path.exists(output_path):
            os.makedirs(output_path)

        np.savez(os.path.join(output_path, 'total_times'), t)
        np.savez(os.path.join(output_path, 'motion_times'), m)
        np.savez(os.path.join(output_path, 'diff_times'), d)
        
        x_range = np.arange(len(t))
        fig = plt.figure(figsize=(12,8))
        ax = fig.gca()
        
        ax.plot(x_range, t, 'r-', label='Total times')
        ax.plot(x_range, m, 'g-', label='Motion Times')
        ax.plot(x_range, d, 'b-', label='Latency Times')

        legend = ax.legend(loc='best', shadow=False)
        frame = legend.get_frame()

        for label in legend.get_texts():
            label.set_fontsize('large')
        for label in legend.get_lines():
            label.set_linewidth(1.5)

        ax.set_title("YuMi Command Times", fontsize=20)
        ax.set_xlabel("Command", fontsize=14)
        ax.set_ylabel("Seconds", fontsize=14)
        fig.savefig(os.path.join(output_path, 'yumi_comm_times.png'), format='png')

        #histograms for all 3 times
        for data, name in ((t, 'total_times'), (m, 'motion_times'), (d, 'latencies')):
            fig = plt.figure(figsize=(12,8))
            ax = fig.gca()
            
            mean = np.mean(data)
            std = np.std(data)
            stats_str = 'mean: {:.3g}\nstd: {:.3g}'.format(mean, std)
        
            props = dict(facecolor='white', alpha=0.5)
            ax.set_title('Histogram of 2-way YuMi Communication Times: {0}'.format(name), fontsize=20)
            ax.set_xlabel('Seconds', fontsize=18)
            ax.set_ylabel('Normalized Count', fontsize=18)

            # place a text box in upper left in axes coords
            ax.text(0.05, 0.95, stats_str, transform=ax.transAxes, fontsize=14,
                    verticalalignment='top', bbox=props)
            h = plt.hist(data, normed=True, bins=30)

            fig.savefig(os.path.join(output_path, 'hist_yumi_2_way_{0}.png'.format(name)), format='png')

        # saving x-direction as plot of time
        times = [0]
        for a_time in total_times:
            times.append(times[-1] + a_time)

        fig = plt.figure(figsize=(12,8))
        ax = fig.gca()
        ax.scatter(times, xs, label='x vs time')
        ax.set_title("Progression of YuMi Arm vs. Time")
        ax.set_xlabel("Time (seconds)")
        ax.set_ylabel("Progression in x-direction (meters)")
        fig.savefig(os.path.join(output_path, 'yumi_progression.png'), format='png')

    def start(self):
        #self._r_motion_sub = rospy.Subscriber(YuMiClient._R_REL, Pose, self._motion_callback_gen('right'))
        #self._r_gripper_sub = rospy.Subscriber(YuMiClient._R_GRIPPER_CLOSE, Bool, self._gripper_callback_gen('right'))
        self._l_motion_sub = rospy.Subscriber(YuMiClient._L_REL, Pose, self._motion_callback_gen('left'))
        self._l_gripper_sub = rospy.Subscriber(YuMiClient._L_GRIPPER_CLOSE, Bool, self._gripper_callback_gen('left'))
                
        rospy.on_shutdown(self._shutdown_hook_gen())
        rospy.spin()

    @staticmethod
    def _ros_to_rigid_transform(ros_pose, from_frame, to_frame):
        translation = np.array([ros_pose.position.x, ros_pose.position.y, ros_pose.position.z])
        rotation = np.array([ros_pose.orientation.w, ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z])
        normed_rotation = rotation / np.linalg.norm(rotation)
        return RigidTransform(translation=translation, rotation=normed_rotation, from_frame=from_frame, to_frame=to_frame)

    @staticmethod
    def _close_enough(pose1, pose2):
        delta_T = pose1.inverse() * pose2

        diff = np.linalg.norm(delta_T.translation) + YuMiClient._ROT_MAG_SCALE * np.linalg.norm(delta_T.rotation)
        if diff < YuMiClient._POSE_DIFF_THRESHOLD:
            return True
        return False

    def _motion_callback(self, arm_name, ros_pose):
        # rate_limiter = self.rate_limiter[arm_name]
        # if not rate_limiter.ok:
        #     return

        T_w_yi = self.T_w_yi[arm_name]

        # turn ros pose into rigid transform
        T_mi_mc = YuMiClient._ros_to_rigid_transform(ros_pose, 'masters_current', 'masters_init')
        
        # scale translations
        T_mi_mc.position = T_mi_mc.position * YuMiClient._MASTERS_TO_YUMI_SCALE

        # transform into YuMi basis
        T_yir_ycr = YuMiClient._T_YIR_MI * T_mi_mc * YuMiClient._T_MC_YCR

        # offset using init pose
        T_w_yc = T_w_yi * self.T_yi_yir[arm_name] * T_yir_ycr * self.T_ycr_yc[arm_name]

        # if YuMiClient._close_enough(T_w_yc, self.last_T_w_yc[arm_name]):
        #     return

        # updating last pose
        self.last_T_w_yc[arm_name] = T_w_yc.copy()
        
        # send pose to YuMi
        pose_q = self._pose_q[arm_name]
        if not pose_q.empty():
            pose_q.get_nowait()
        pose_q.put(T_w_yc)

    def _motion_callback_gen(self, arm_name):
        return lambda pose: self._motion_callback(arm_name, pose)
        
    def _gripper_callback(self, arm_name, closed):
        arm = getattr(self.yumi, arm_name)
        if closed:
            arm.close_gripper()
        else:
            arm.open_gripper()
        
    def _gripper_callback_gen(self, arm_name):
        return lambda bool: self._gripper_callback(arm_name, bool.data)

if __name__ == '__main__':
    logging.getLogger().setLevel(YMC.LOGGING_LEVEL)
    parser = argparse.ArgumentParser(description='YuMi client for masters tele-op')
    parser.add_argument('output_path', help='the output path of data recorded')
    parser.add_argument('-z', '--zone', type=str, default='fine', help='zone of the robot')
    args = parser.parse_args()

    yumi_client = YuMiClient(args.output_path, args.zone)
    yumi_client.start()