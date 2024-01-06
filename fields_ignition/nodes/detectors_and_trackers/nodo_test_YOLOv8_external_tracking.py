#!/usr/bin/env python3

# imports
import rospy
from sensor_msgs.msg import CompressedImage
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

# clone repository with tracker and tracker evaluator - after running the node for the first time, run `pip install --upgrade sentry-sdk`
subprocess.run(["git", "clone", "--recurse-submodules", "https://github.com/roxana-garderes/yolov8_tracking.git"])
subprocess.run(["pip", "install", "-r", "yolov8_tracking/requirements.txt"])
os.makedirs("yolov8_tracking/weights", exist_ok=True)
subprocess.run(["git", "clone", "https://github.com/JonathonLuiten/TrackEval.git", "yolov8_tracking/val_utils"])
subprocess.run(["cp", "-r", "yolov8_tracking/boxmot", "yolov8_tracking/examples"])


# global variables
TRACKING_METHOD = "bytetrack"
YOLO_WEIGHTS = "weights_YOLOv8.pt"
bridge = CvBridge()
YOLOv8_model = None

# when the nodes ends track the apples and evaluate the tracking
def exit_handler():
    print('Running tracker and tracker evaluator...')
    
    SOURCE = "detected_images_YOLOv8"
    subprocess.run(["python3", "yolov8_tracking/examples/track.py", "--yolo-model", YOLO_WEIGHTS, "--tracking-method", TRACKING_METHOD, "--source", SOURCE, "--save", "--hide-label", "--hide-conf"]) 

    

# saves an image and returns its name
def save_image(save_path, saved_image_name, global_frame):
    current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S%f") + "_"
    cv2.imwrite(save_path + '/' + current_time + saved_image_name, global_frame)
    return current_time + saved_image_name

# callback function
def process_data(data):
    try:
        global_frame = bridge.compressed_imgmsg_to_cv2(data)
        # save image with no annotations first
        saved_image_name = save_image("detected_images_YOLOv8", "detected.jpg", global_frame)

        # pdb.set_trace()

        # test
        # SOURCE = "detected_images_YOLOv8/" + saved_image_name
        # subprocess.run(["python3", "yolov8_tracking/examples/track.py", "--yolo-model", YOLO_WEIGHTS, "--tracking-method", TRACKING_METHOD, "--source", SOURCE, "--save", "--hide-label", "--hide-conf"])
        # ---------------

    except CvBridgeError as e:
        raise(e)


# main function
if __name__ == '__main__':

    bridge = CvBridge()
    YOLOv8_model = YOLO(YOLO_WEIGHTS)

    rospy.init_node('test_node')
    
    # read from simulation
    # sub = rospy.Subscriber("/costar_husky_sensor_config_1/left/image_raw/compressed", CompressedImage, process_data, queue_size = 10) 
    
    # read from bag
    sub = rospy.Subscriber("/zed_lateral/zed_lateral/left/image_rect_color/compressed", CompressedImage, process_data, queue_size = 10) 

    rospy.loginfo('test node has been started...')

    # when the node is killed, run the tracker and the tracker evaluator
    rospy.on_shutdown(exit_handler)
    
    rospy.spin() #blocks until node is shutdown, Yields activity on other threads