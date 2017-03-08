from yumi_teleop import load_records, load_poses, load_joints, load_images
import IPython

if __name__ == '__main__':
    # CSV Model for all teleop trials
    records = load_records('t')

    # get a single row of records from the csv
    a_trial_record = record.get_by_cols({
        'supervisor': 'Jacky',
        'demo_name': 'scoop_demo',
        'trial_num': 4
    })
    
    trial_path = a_trial_record['trial_path']
    
    # poses of left arm
    left_poses = load_poses(trial_path, 'left')
    print 'left poses shape', left_poses.shape
    
    # joints of left arm
    left_joints = load_joints(trial_path, 'left')
    print 'left joints shape', left_joints.shape
    
    # webcam frames
    webcam = load_images(trial_path, 'webcam')
    print 'webcam shape', webcam.shape

    # kinect frames
    kinect = load_images(trial_path, 'kinect_depth')
    print 'kinect shape', kinect.shape    
    
    IPython.embed()
    exit(0)