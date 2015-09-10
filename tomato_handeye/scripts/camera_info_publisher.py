#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import yaml
import roslib
from sensor_msgs.msg import CameraInfo
import rospy
import rospkg

class CameraInfoPublisher: 
# Callback of the ROS subscriber. 
    def callback(self, data):
        cam_info_org = data
        self.cam_info.header = cam_info_org.header
        publisher.publish()

    def __init__(self, camera_name):
        self.cam_info_org = 0
        # yamlファイルを読み込んでCameraInfoを返す
        file_name  = rospy.get_param(
            '~file_name',
            rospack.get_path('tomato_handeye') + '/calib/cal.yml')
        self.cam_info = parse_yaml(file_name)
        cinfo_topic = "/" + camera_name + "/camera_info"
        # Timestampを合わせるためにsubする必要あり
        rospy.Subscriber("/null/camera_info", CameraInfo, self.callback)
        self.cinfo_pub = rospy.Publisher(cinfo_topic,CameraInfo) 

    def publish(self):
        '''
        now = rospy.Time.now()
        self.cam_info.header.stamp = now
        '''
        self.cinfo_pub.publish(self.cam_info)  

def parse_yaml(filename):
    stream = file(filename, 'r')
    calib_data = yaml.load(stream)
    cam_info = CameraInfo()
    cam_info.width = calib_data['image_width']
    cam_info.height = calib_data['image_height']
    cam_info.K = calib_data['camera_matrix']['data']
    cam_info.D = calib_data['distortion_coefficients']['data']
    cam_info.R = calib_data['rectification_matrix']['data']
    cam_info.P = calib_data['projection_matrix']['data']
    cam_info.distortion_model = calib_data['distortion_model']
    cam_info.binning_x = calib_data['binning_x']
    cam_info.binning_y = calib_data['binning_y']
    cam_info.roi.x_offset = calib_data['roi']['x_offset']
    cam_info.roi.y_offset = calib_data['roi']['y_offset']
    cam_info.roi.height = calib_data['roi']['height']
    cam_info.roi.width = calib_data['roi']['width']
    cam_info.roi.do_rectify = calib_data['roi']['do_rectify']
    
    return cam_info

if __name__ == '__main__': 
    argv = rospy.myargv(sys.argv)
    rospy.init_node("camera_info_publisher")
    rospack = rospkg.RosPack()
    publisher = CameraInfoPublisher('camera_out')

    while not rospy.is_shutdown():
        rospy.sleep(rospy.Duration(.1))
