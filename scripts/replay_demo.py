'''
Script to replay a recorded demonstration.
Author: Jacky
'''
import argparse, logging, os
from joblib import load

from core import YamlConfig, CSVModel, DataStreamSyncer, DataStreamRecorder
from yumipy import YuMiRobot, YuMiSubscriber
from yumi_teleop import DemoWrapper, Sequence
from time import sleep

import IPython

def playback(args):
    cfg = YamlConfig(args.config_path)
    demo_name = args.demo_name
    supervisor = args.supervisor
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
        'trial_num': trial_num,
        'supervisor': supervisor
    })
    trial_path = demo_record['trial_path']
    demo_host_cfg = YamlConfig(os.path.join(trial_path, 'demo_config.yaml'))

    # parse demo trajectory
    # TODO: enforce fps
    fps = demo_host_cfg['fps']

    _, left_data = zip(*load(os.path.join(trial_path, '{0}_left.jb'.format(cfg['mode']))))
    _, right_data = zip(*load(os.path.join(trial_path, '{0}_right.jb'.format(cfg['mode']))))
    _, gripper_left_evs = zip(*load(os.path.join(trial_path, 'grippers_bool_left.jb')))
    _, gripper_right_evs = zip(*load(os.path.join(trial_path, 'grippers_bool_right.jb')))

    sequences = {
        'left': Sequence(left_data),
        'right': Sequence(right_data),
        'gripper_left': Sequence(gripper_left_evs),
        'gripper_right': Sequence(gripper_right_evs)
    }

    subsample_factor = cfg['subsample']
    subsampled_sequences = {
        'left': sequences['left'].subsampler(subsample_factor),
        'right': sequences['right'].subsampler(subsample_factor),
        'gripper_left': sequences['gripper_left'].subsampler(subsample_factor, retain_features=True),
        'gripper_right': sequences['gripper_right'].subsampler(subsample_factor, retain_features=True)
    }

    N = min([len(seq.data) for seq in subsampled_sequences.values()]) # avoids 1-off errors

    # processing time steps where zoning should be set to fine
    gripper_zoning = [None for _ in range(N)]
    for t in range(N-1):
        if subsampled_sequences['gripper_left'].data[t] != None or \
            subsampled_sequences['gripper_right'].data[t] != None:
            if t == 0:
                y.set_z('fine')
            else:
                gripper_zoning[t-1] = 'fine'
            gripper_zoning[t+1] = cfg['z']

    # perform setup motions
    logging.info("Loading demo and performing setups.")
    y.reset_home()
    y.open_grippers()
    demo_path = os.path.join(trial_path, '{0}.py'.format(demo_name))
    demo_obj = DemoWrapper.load(demo_path, y)
    demo_obj.setup()

    # record torque and other debug data if needed
    if cfg['record_torque']['use']:
        ysub = YuMiSubscriber()
        ysub.start()
        data_torque_left = DataStreamRecorder('torques_left', ysub.left.get_torque, cache_path=cfg['cache_path'], save_every=cfg['save_every'])
        data_torque_right = DataStreamRecorder('torques_right', ysub.right.get_torque, cache_path=cfg['cache_path'], save_every=cfg['save_every'])
        syncer = DataStreamSyncer([data_torque_left, data_torque_right], fps)
        syncer.start()
        sleep(0.5)
        syncer.pause()
        syncer.flush()
        syncer.resume(reset_time=True)

        torque_model = CSVModel.get_or_create(
            os.path.join(cfg['data_path'], 'playback_torques_record.csv'),
            [
                ('supervisor', 'str'),
                ('demo_name', 'str'),
                ('trial_num', 'int'),
                ('playback_num', 'int'),
                ('playback_path', 'str')
            ]
        )

        last_torque_record = torque_model.get_by_cols({
                                                    'demo_name': demo_name,
                                                    'trial_num': trial_num,
                                                    'supervisor': supervisor
                                                }, direction=-1)
        if last_torque_record == None:
            playback_num = 1
        else:
            playback_num = last_torque_record['playback_num'] + 1

        playback_path = os.path.join(trial_path, 'playback_torques')
        if not os.path.exists(playback_path):
            os.makedirs(playback_path)

    # perform trajectory
    logging.info("Playing trajectory")
    for t in range(N):
        left_item = subsampled_sequences['left'].data[t][1]
        right_item = subsampled_sequences['right'].data[t][1]
        gripper_left_item = subsampled_sequences['gripper_left'].data[t]
        gripper_right_item = subsampled_sequences['gripper_right'].data[t]

        if cfg['mode'] == 'poses':
            y.left.goto_pose(left_item, relative=True, wait_for_res=False)
            y.right.goto_pose(right_item, relative=True, wait_for_res=True)
        else:
            y.left.goto_state(left_item, wait_for_res=False)
            y.right.goto_state(right_item, wait_for_res=True)

        if gripper_left_item != None and gripper_right_item != None:
            getattr(y.left, gripper_left_item)(wait_for_res=False)
            getattr(y.right, gripper_right_item)(wait_for_res=True)
        elif gripper_left_item != None:
            getattr(y.left, gripper_left_item)()
        elif gripper_right_item != None:
            getattr(y.right, gripper_right_item)()

        z = gripper_zoning[t]
        if z is not None:
            logging.info("Setting zone to {0}".format(z))
            y.set_z(z)

    if cfg['record_torque']['use']:
        syncer.pause()

        data_torque_left.save_data(playback_path)
        data_torque_right.save_data(playback_path)

        torque_model.insert({
            'supervisor': supervisor,
            'demo_name': demo_name,
            'trial_num': trial_num,
            'playback_num': playback_num,
            'playback_path': playback_path
        })

        ysub.stop()
        syncer.stop()

    # perform takedown motions
    logging.info("Taking down..")
    y.set_v(cfg['v'])
    y.set_z(cfg['z'])
    demo_obj.takedown()

    y.reset_home()
    y.open_grippers()

    y.stop()

if __name__ == '__main__':
    logging.getLogger().setLevel(logging.INFO)
    parser = argparse.ArgumentParser(description='YuMi Teleop Host')
    parser.add_argument("demo_name", type=str, help='name of demo')
    parser.add_argument("supervisor", type=str, help='name of supervisor')
    parser.add_argument("trial_num", type=int, help='trial num')
    parser.add_argument('-c', '--config_path', type=str, default='cfg/replay_config.yaml', help='path to config file')

    args = parser.parse_args()
    playback(args)
