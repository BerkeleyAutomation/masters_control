'''
Script to record a list of msgs from ros that can be replayed for debugging/testing.
Author: Jacky
'''
import argparse
import yaml
import os
import logging

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose

class RosRecorder:

    @staticmethod
    def play(cfg, output_path):
        topics = cfg['topics']
        return
    
    @staticmethod
    def record(cfg, output_path):
        topics = cfg['topics']
        
        
        return

if __name__ == '__main__':
    logging.getLogger().setLevel(logging.INFO)
    
    parser = argparse.ArgumentParser(description='ROS states recorder')
    parser.add_argument('mode')
    parser.add_argument('config_filename')
    parser.add_argument('output_path')
    args = parser.parse_args()
    
    with open(args.config_filename) as config_file:
        cfg = yaml.safe_load(config_file)
        
    if args.mode not in ("record", "play"):
        logging.error("Unknown mode: {0}. Has to be 'record' or 'play'. ".format(args.mode))
        return
        
    getattr(RosRecorder, args.mode)(cfg, args.output_path)
        
    
    
    