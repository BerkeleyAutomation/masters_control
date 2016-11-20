"""
Experiment Logger class for teleop demonstrations
Author: Jacky Liang
"""
from core import ExperimentLogger, CSVModel
import os, cv2, logging
from joblib import load
from perception import write_video

_LOGGING_LEVEL = logging.DEBUG

class TeleopExperimentLogger(ExperimentLogger):

    RECORDS_HEADERS_TYPES = (
            ('experiment_id','str'),
            ('demo_name','str'),
            ('trial_num', 'int'),
            ('path_trial','str')
        )

    def __init__(self, experiment_root_path, supervisor):
        self.supervisor = supervisor
        super(TeleopExperimentLogger, self).__init__(experiment_root_path)

        demo_records_model_path = os.path.join(self.experiment_root_path, 'demo_records.csv')
        self._demo_records_model = CSVModel.get_or_create(demo_records_model_path, TeleopExperimentLogger.RECORDS_HEADERS_TYPES)

    @property
    def experiment_meta_headers_types(self):
        """:obj:`tuple` of :obj:`tuple` of :obj:`str` : A tuple of tuples, each
        of which maps string header names to string type names.
        """
        return (
                ('session_id','str'),
                ('supervisor','str'),
        )

    @property
    def experiment_meta_dict(self):
        """:obj:`dict` : The metadata of the experiment as a dictionary.
        """
        return {
                'session_id': self.id,
                'supervisor': self.supervisor
                }

    def save_demo_data(self, demo_name, setup_path, config_path, data_streamers, fps):

        logging.getLogger().setLevel(_LOGGING_LEVEL)
        last_demo_record = self._demo_records_model.get_by_col_last('demo_name', demo_name)
        if last_demo_record == None:
            trial_num = 1
        else:
            trial_num = last_demo_record['trial_num'] + 1

        trial_num_str = str(trial_num)
        trial_dirs = [demo_name, trial_num_str]
        self.construct_internal_dirs(trial_dirs, realize=True)
        trial_path = self.dirs_to_path(trial_dirs)

        # saving setup demo file
        self.copy_to_dir(setup_path, trial_dirs)
        self.copy_to_dir(config_path, trial_dirs)

        # callback to save video
        def save_video():
            webcam_data_path = os.path.join(trial_path, 'webcam.jb')
            while not os.path.isfile(webcam_data_path):
                sleep(1e-2)
            webcam_data = load(webcam_data_path)
            frames = [data[1] for data in webcam_data]
            write_video(frames, os.path.join(trial_path, 'webcam.avi'), fps=fps)

        # saving all data
        for data_streamer in data_streamers:
            if data_streamer.name == 'webcam':
                data_streamer.save_data(trial_path, cb=save_video)
            else:
                data_streamer.save_data(trial_path)

        self._demo_records_model.insert({
            'experiment_id': self.id,
            'demo_name': demo_name,
            'trial_num': trial_num,
            'path_trial': trial_path,
        })
