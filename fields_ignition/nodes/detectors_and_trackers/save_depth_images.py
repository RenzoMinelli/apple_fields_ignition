#!/usr/bin/env python3

# imports
import rospy
from sensor_msgs.msg import CompressedImage
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Imu
import time as Time
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import datetime
import pdb
import subprocess
import os

# callback function
def process_data(data):
    try:
        # debugger
        pdb.set_trace()
        global_frame = bridge.imgmsg_to_cv2(data)
        # save the image with the timestamp of the message, NOT the current time
        current_time = data.header.sec
        print("saving image in detected_images_depth_data/" + str(current_time) + "_detected.jpg")
        cv2.imwrite("detected_images_depth_data/" + str(current_time) + "_detected.jpg", global_frame)
    except CvBridgeError as e:
        raise(e)


# main function
if __name__ == '__main__':

    bridge = CvBridge()

    rospy.init_node('test_node')
    
    # read from simulation
    sub = rospy.Subscriber("/costar_husky_sensor_config_1/disparity", DisparityImage, process_data, queue_size = 10)

    # read from bag
    # sub = rospy.Subscriber("/zed_lateral/zed_lateral/left/image_rect_color/compressed", CompressedImage, process_data, queue_size = 10) 

    rospy.loginfo('test node has been started...')

    rospy.spin() #blocks until node is shutdown, Yields activity on other threads
