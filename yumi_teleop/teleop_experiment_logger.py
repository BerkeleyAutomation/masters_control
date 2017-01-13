"""
Experiment Logger class for teleop demonstrations
Author: Jacky Liang
"""
import os, cv2, logging
from time import sleep, time
import requests
from core import ExperimentLogger, CSVModel
from constants import VIDEO_SERVICE_PORT

class TeleopExperimentLogger(ExperimentLogger):

    RECORDS_HEADERS_TYPES = [
            ('experiment_id','str'),
            ('demo_name','str'),
            ('supervisor', 'str'),
            ('trial_num', 'int'),
            ('trial_path','str'),
        ]

    def __init__(self, experiment_root_path, supervisor):
        self.supervisor = supervisor
        super(TeleopExperimentLogger, self).__init__(experiment_root_path, sub_experiment_dirs=False)

        demo_records_model_path = os.path.join(self.experiment_root_path, 'demo_records.csv')
        self._demo_records_model = CSVModel.get_or_create(demo_records_model_path, TeleopExperimentLogger.RECORDS_HEADERS_TYPES)

    @property
    def experiment_meta_headers(self):
        """:obj:`tuple` of :obj:`tuple` of :obj:`str` : A list of tuples, each
        of which maps string header names to string type names.
        """
        return [
                ('session_id','str'),
                ('supervisor','str'),
        ]

    @property
    def experiment_meta_data(self):
        """:obj:`dict` : The metadata of the experiment as a dictionary.
        """
        return {
                'session_id': self.id,
                'supervisor': self.supervisor
                }

    def save_demo_data(self, demo_name, supervisor, save_file_paths, data_streamers, fps):
        last_demo_record = self._demo_records_model.get_by_cols({
                                                                'demo_name': demo_name,
                                                                'supervisor': supervisor
                                                                }, direction=-1)
        if last_demo_record == None:
            trial_num = 1
        else:
            trial_num = last_demo_record['trial_num'] + 1

        trial_num_str = str(trial_num)
        trial_dirs = [supervisor, demo_name, trial_num_str]
        self.construct_internal_dirs(trial_dirs, realize=True)
        trial_path = self.dirs_to_path(trial_dirs)

        logging.info("Saving data to {0}".format(trial_path))

        # saving files
        for path in save_file_paths:
            self.copy_to_dir(path, trial_dirs)

        # callback to save video
        def save_video():
            _ = requests.get("http://localhost:{0}/render_video?trial_path={1}&fps={2}".format(VIDEO_SERVICE_PORT, trial_path, fps))
            logging.info("Finished sending request to render video.")

        # saving all data
        def notify_complete(name):
            return lambda : logging.info("Finished saving {0}".format(name))
        for data_streamer in data_streamers:
            if data_streamer.name == 'webcam':
                data_streamer.save_data(trial_path, cb=save_video, concat=False)
            else:
                data_streamer.save_data(trial_path, cb=notify_complete(data_streamer.name))

        self._demo_records_model.insert({
            'experiment_id': self.id,
            'demo_name': demo_name,
            'supervisor': supervisor,
            'trial_num': trial_num,
            'trial_path': trial_path,
        })
