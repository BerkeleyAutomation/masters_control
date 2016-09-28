#!/usr/bin/env python
import numpy as np
import rospy
from cv_bridge import CvBridge
import skimage as sk
from sensor_msgs.msg import Image
from alan.rgbd import Kinect2Sensor
import argparse

_ROSTOPIC = 'YuMi_Kinect'
_NODENAME = 'YuMi_Kinect_Publisher'

def kinect_publisher(device_num, fps):
    pub = rospy.Publisher(_ROSTOPIC, Image, queue_size=1)
    rospy.init_node(_NODENAME, anonymous=True)
    rate = rospy.Rate(fps)

    kinect = Kinect2Sensor(device_num=device_num)
    kinect.start()

    bridge = CvBridge()

    rospy.loginfo("Starting kinect stream on device {0}".format(device_num))
    while not rospy.is_shutdown():
        color_im, _, __ = kinect.frames(skip_registration=True)
        raw_im = color_im.raw_data
        #raw_im = sk.transform.rescale(raw_im, 0.5).astype('uint8')
        raw_im = raw_im[500:,500:,:]
        im_msg = bridge.cv2_to_imgmsg(raw_im, 'rgb8')
        pub.publish(im_msg)
        rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Capture a set of test images from the Kinect2')
    parser.add_argument('-d', '--device_num', type=int, default=0, help='device number of the kinect sensor')
    parser.add_argument('-f', '--fps', type=int, default=30, help='fps of the streaming')
    args = parser.parse_args()

    try:
        kinect_publisher(args.device_num, args.fps)
    except rospy.ROSInterruptException:
        pass