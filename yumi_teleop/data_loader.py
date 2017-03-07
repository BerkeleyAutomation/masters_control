'''
Utility functions to load yumi teleop recorded data into numpy arrays
Author: Jacky Liang
'''

import os
from joblib import load
import numpy as np
from core import CSVModel, RigidTransform

ROOT_PATH = '/mnt/hdd/data/'

def load_records(mode):
    if mode not in ('k', 't'):
        raise ValueError("Can only accept k (Kinesthetic) or t (Teleop) as modes. Got {}".format(mode))

    path = os.path.join(ROOT_PATH, 'yumi_{}'.format('kinesthetic' if mode == 'k' else 'teleop'), 'demo_records.csv')
    records = CSVModel.load(path)
    return records

def load_registration_tf(trial_path, device):
    if device not in ('webcam', 'primesense'):
        raise ValueError("Can only accept webcam or primesense. Got {}".format(device))
    return RigidTransform.load(os.path.join(trial_path, '{}_overhead_to_world.tf'.format(device)))

def load_poses(trial_path, arm_name):
    '''
    Returns n by 6 numpy array. n is # of time steps.
    the first 3 cols are x-y-z translation, last 3 are sxyz euler angles
    '''
    if arm_name not in ('left', 'right'):
        raise ValueError("Arm name can only be left or right. Got {}".format(arm_name))
    poses_raw = load(os.path.join(trial_path, 'poses_{}.jb'.format(arm_name)))
    poses = [x[1][1] for x in poses_raw]
    poses_lst = [np.r_[p.translation, p.euler] for p in poses]
    return np.array(poses_lst)

def load_joints(trial_path, arm_name):
    '''
    Returns n by 7 numpy array. n is # of time steps. the ith col is the angle of the ith joint
    '''
    if arm_name not in ('left', 'right'):
        raise ValueError("Arm name can only be left or right. Got {}".format(arm_name))
    states_raw = load(os.path.join(trial_path, 'states_{}.jb'.format(arm_name)))
    states = [x[1][1] for x in states_raw]
    joints_lst = [s.joints for s in states]
    return np.array(joints_lst)

def load_webcam(trial_path):
    '''
    Returns n by 480 by 680 by 3 np array. color channels are RGB
    '''
    webcam_data_path = os.path.join(trial_path, 'webcam')

    all_chunks = os.listdir(webcam_data_path)
    all_chunks.sort()

    webcam_data = []
    for i in range(1, len(all_chunks)):
        webcam_data.extend(load(os.path.join(webcam_data_path, '{}.jb'.format(i))))

    frames = [x[1].data for x in webcam_data]
    return np.array(frames)

def load_primesense(trial_path):
    '''
    Returns n by 640 by 480 np array
    '''
    ps_data_path = os.path.join(trial_path, 'primesense_depth')

    all_chunks = os.listdir(ps_data_path)
    all_chunks.sort()

    ps_data = []
    for i in range(1, len(all_chunks)):
        ps_data.extend(load(os.path.join(ps_data_path, '{}.jb'.format(i))))

    frames = [data[1] for data in ps_data]
    return np.array(frames)
