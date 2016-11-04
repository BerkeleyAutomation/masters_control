"""
Experiment Logger class for teleop demonstrations
Author: Jacky Liang
"""
from core import ExperimentLogger, CSVModel
from joblib import dump
import cv2

class TeleopExperimentLogger(ExperimentLogger):

    RECORDS_HEADERS_TYPES = (
            ('experiment_id','str'),
            ('demo_name','str'),
            ('trial_num', 'int')
            ('trial_duration','float'),
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
                ('time_started','str'),
                ('time_stopped','str'),
                ('duration','float'),
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

    def save_demo_data(self, demo_name, setup_path, webcam_data, kinect_depth_data,
                            kinect_color_data, motion_data):

        last_demo_record = self.master_record.get_by_col_last('demo_name', demo_name)
        if last_demo_record == None:
            trial_num = 1
        else:
            trial_num = last_demo_record['trial_num'] + 1

        trial_num_str = str(trial_num)
        trial_dirs = [demo_name, trial_num_str]
        self.construct_internal_dirs(trial_dirs, realize=True)
        trial_path = dirs_to_path(trial_dirs)

        # saving setup demo file
        self.copy_to_dir(setup_path, trial_dirs)

        # saving video
        fourcc = cv2.cv.CV_FOURCC(*'XVID')
        writer = cv2.VideoWriter(os.path.join(trial_path, 'video.avi'), fourcc, 30, tuple(webcam_data[0].shape))
        for frame in webcam_data:
            writer.write(frame)
        writer.release()       

        # saving visual data
        dump(os.path.join(trial_path, "kinect_depth_data.jb"), kinect_depth_data, compress=3)
        dump(os.path.join(trial_path, "kinect_color_data.jb"), kinect_color_data, compress=3)
        dump(os.path.join(trial_path, "webcam_data.jb"), webcam_data, compress=3)

        # saving motion data
        for side, data in motion_data:
            self.construct_internal_dirs(trial_dirs + [side], realize=True)
            side_path = os.path.join(trial_path, side)
            for key, val in data:
                dump(os.path.join(side_path, "{0}.jb".format(key)), val, compress=3)
