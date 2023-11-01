#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
import time as Time
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import datetime

bridge = CvBridge()
YOLOv8_model = None


def process_data(data):
    #rospy.loginfo(data)

    try:
        global_frame = bridge.compressed_imgmsg_to_cv2(data)
        results = YOLOv8_model.predict(global_frame, stream=True)
        print("results: ", results)
        
        for result in results:                                         # iterate results
            boxes = result.boxes.cpu().numpy()                         # get boxes on cpu in numpy
            for box in boxes:                                          # iterate boxes
                r = box.xyxy[0].astype(int)                            # get corner points as int
                print(r)                                               # print boxes
                cv2.rectangle(global_frame, r[:2], r[2:], (255, 255, 255), 2)   # draw boxes on img

        # Save the image
        current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S%f")
        cv2.imwrite("detected_images_YOLOv8/" + current_time + "_detected.jpg", global_frame)
    except CvBridgeError as e:
        throw(e)


if __name__ == '__main__':

    bridge = CvBridge()
    YOLOv8_model = YOLO('weights_YOLOv8.pt')

    rospy.init_node('test_node')    
    sub = rospy.Subscriber("/costar_husky_sensor_config_1/left/image_raw/compressed", CompressedImage, process_data, queue_size = 10)
    #sub2 = rospy.Subscriber("/zed_lateral/zed_lateral/imu/data", Imu, process_data)
    #sub3 = rospy.Subscriber("/zed_front/zed_node_front/imu/data", Imu, process_data)

    rospy.loginfo('test node has been started...')
    rospy.spin() #blocks until node is shutdown, Yields activity on other threads