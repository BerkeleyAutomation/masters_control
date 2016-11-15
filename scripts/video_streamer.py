#!/usr/bin/env python
'''
Streams given Kinect2Sensor or OpenCVCameraSensor to ROS
Author: Jacky
'''
import rospy
from cv_bridge import CvBridge
from skimage.transform import rescale
from sensor_msgs.msg import Image
from perception import Kinect2Sensor, Kinect2PacketPipelineMode, OpencvCameraSensor
import argparse

_ROSTOPIC = 'sensor_{0}_{1}'
_NODENAME = 'sensor_{0}_{1}_publisher'

def publish_video(device_num, device_type, fps):
    rostopic = _ROSTOPIC.format(device_type, device_num)
    nodename = _NODENAME.format(device_type, device_num)

    pub = rospy.Publisher(rostopic, Image, queue_size=1)
    rospy.init_node(nodename, anonymous=True)
    rate = rospy.Rate(fps)

    def _get_raw_im_gen():
        device = None
        if device_type.lower() == 'kinect':
            device = Kinect2Sensor(device_num=device_num, packet_pipeline_mode=Kinect2PacketPipelineMode.OPENGL)
        elif device_type.lower() == 'webcam':
            device = OpencvCameraSensor(device_num)
        else:
            raise ValueError("Unknown device type! Only takes kinect or webcam. Got: {0}".format(device_type))

        device.start()

        def get_raw_im():
            if device_type == 'kinect':
                color_im, _, _ = device.frames(skip_registration=True)
                raw_im = rescale(color_im.raw_data, 0.3, preserve_range=True).astype('uint8') # rescaling 1080p image
            else:
                raw_im = device.frames().raw_data
            return raw_im

        return get_raw_im

    get_raw_im = _get_raw_im_gen()
    bridge = CvBridge()

    rospy.loginfo("Rostopic: {0}".format(rostopic))
    rospy.loginfo("Streaming {0} device {1} at {2} fps".format(device_type, device_num, fps))
    while not rospy.is_shutdown():
        raw_im = get_raw_im()
        im_msg = bridge.cv2_to_imgmsg(raw_im, 'rgb8')

        pub.publish(im_msg)
        rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Stream a video device, either kinect2 or webcam, to ROS')
    parser.add_argument('-d', '--device_num', type=int, default=1, help='device number')
    parser.add_argument('-f', '--fps', type=int, default=24, help='fps')
    parser.add_argument('-t', '--type', type=str, default='kinect', help="Either kinect or webcam")
    args = parser.parse_args()

    try:
        publish_video(args.device_num, args.type, args.fps)
    except rospy.ROSInterruptException:
        pass
