from yumi_teleop import load_records, load_poses, load_joints, load_images
import IPython
import numpy as np
import os
import sys

if __name__ == '__main__':
    # CSV Model for all teleop trials
    records = load_records('t')

    # get all successful trials
    all_trials = records.get_rows_by_cols({
        'supervisor': 'Jeff_Kinect',
        'success': True,
        'demo_name': 'scoop_demo'
    })

    for i, trial in enumerate(all_trials):
        if trial['supervisor'] == 'Jeff_Kinect' and \
           trial['demo_name'] == 'scoop_demo' and \
           trial['success'] == True:

            # get path to trial data
            trial_path = trial['trial_path']

            # tensor save dir
            tensor_path = os.path.join(trial_path, 'tensors')
            if not os.path.exists(tensor_path):
                os.mkdir(tensor_path)

            print 'Loading data for trial', trial_path
            # poses of left arm
            left_poses = load_poses(trial_path, 'left')
            left_poses_filename = os.path.join(tensor_path, 'left_poses.npz')
            np.savez_compressed(left_poses_filename, left_poses)
            print 'left poses shape', left_poses.shape
            
            # joints of left arm
            left_joints = load_joints(trial_path, 'left')
            left_joints_filename = os.path.join(tensor_path, 'left_joints.npz')
            np.savez_compressed(left_joints_filename, left_joints)
            print 'left joints shape', left_joints.shape

            # poses of left arm
            right_poses = load_poses(trial_path, 'right')
            right_poses_filename = os.path.join(tensor_path, 'right_poses.npz')
            np.savez_compressed(right_poses_filename, right_poses)
            print 'right poses shape', right_poses.shape
            
            # joints of left arm
            right_joints = load_joints(trial_path, 'right')
            right_joints_filename = os.path.join(tensor_path, 'right_joints.npz')
            np.savez_compressed(right_joints_filename, right_joints)
            print 'right joints shape', right_joints.shape
            
            # webcam frames
            webcam = load_images(trial_path, 'webcam')
            webcam_filename = os.path.join(tensor_path, 'webcam.npz')
            np.savez_compressed(webcam_filename, webcam)
            print 'webcam shape', webcam.shape
