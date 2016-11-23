'''
Script to replay a recorded demonstration.
Author: Jacky
'''
import argparse, logging, os
from joblib import load

from core import YamlConfig, CSVModel
from yumipy import YuMiRobot
from yumi_teleop import load_demo_class, Trajectory
from time import sleep

import IPython

def playback(args):
    cfg = YamlConfig(args.config_path)
    demo_name = args.demo_name
    trial_num = args.trial_num

    if cfg['mode'] not in ('poses', 'states'):
        y.stop()
        raise ValueError("Unknown playback mode! Only accepts 'poses' or 'joints'. Got {0}".format(cfg['mode']))

    # init robot
    logging.info("Init robot.")
    y = YuMiRobot()
    y.set_v(cfg['v'])
    y.set_z(cfg['z'])

    # load demo data
    demo_records = CSVModel.load(os.path.join(cfg['data_path'], 'demo_records.csv'))
    demo_record = demo_records.get_by_cols({
        'demo_name': demo_name,
        'trial_num': trial_num
    })
    path_trial = demo_record['path_trial']
    demo_host_cfg = YamlConfig(os.path.join(path_trial, 'host_config.yaml'))

    # parse demo trajectory
    # TODO: gripper events
    # TODO: enforce fps

    fps = demo_host_cfg['fps']
    times, left_data = zip(*load(os.path.join(path_trial, 'motion_{0}_left.jb'.format(cfg['mode']))))
    _, right_data = zip(*load(os.path.join(path_trial, 'motion_{0}_right.jb'.format(cfg['mode']))))

    traj = Trajectory(times, zip(left_data, right_data))
    subsampled_traj = traj.subsampler(cfg['subsample'])

    # perform setup motions
    logging.info("Loading demo and performing setups.")
    y.reset_home()
    y.open_grippers()
    demo_path = os.path.join(path_trial, '{0}.py'.format(demo_name))
    demo_obj = load_demo_class(demo_path, y)
    demo_obj.setup()

    # perform trajectory
    logging.info("Playing trajectory")
    for data in subsampled_traj:
        if cfg['mode'] == 'poses':
            y.left.goto_pose(data[0][1], relative=True, wait_for_res=False)
            y.right.goto_pose(data[1][1], relative=True, wait_for_res=True)
        else:
            y.left.goto_state(data[0][1], wait_for_res=False)
            y.right.goto_state(data[1][1], wait_for_res=True)

    # perform takedown motions
    logging.info("Taking down..")
    demo_obj.takedown()

    y.reset_home()
    y.open_grippers()

    y.stop()

if __name__ == '__main__':
    logging.getLogger().setLevel(logging.DEBUG)
    parser = argparse.ArgumentParser(description='YuMi Teleop Host')
    parser.add_argument("demo_name", type=str, help='name of demo')
    parser.add_argument("trial_num", type=int, help='trial num of demo')
    parser.add_argument('-c', '--config_path', type=str, default='cfg/replay_config.yaml', help='path to config file')

    args = parser.parse_args()
    playback(args)
