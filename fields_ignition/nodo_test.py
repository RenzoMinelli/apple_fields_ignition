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
YOLOv5_model = torch.hub.load('ultralytics/yolov5', 'custom', path='weights.pt')

def process_data(data):
    #rospy.loginfo(data)

    try:
        global_frame = bridge.compressed_imgmsg_to_cv2(data)
        results = YOLOv5_model(global_frame)
        print("results: ", results)
        
        for label, score, box in zip(results.pred[0][:, -1], results.pred[0][:, -2], results.pred[0][:, :-2]):
            x1, y1, x2, y2 = box
            # Draw a bounding box rectangle and label on the image
            cv2.rectangle(global_frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
            cv2.putText(global_frame, f'{int(label)} {score:.2f}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Save the image
        current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        cv2.imwrite("detected_images_YOLOv5/" + current_time + "_detected.jpg", global_frame)
    except CvBridgeError as e:
        throw(e)


if __name__ == '__main__':

    bridge = CvBridge()
    YOLOv5_model = torch.hub.load('ultralytics/yolov5', 'custom', path='weights.pt')

    rospy.init_node('test_node')    
    sub = rospy.Subscriber("/zed_lateral/zed_lateral/left/image_rect_color/compressed", CompressedImage, process_data, queue_size = 1)
    #sub2 = rospy.Subscriber("/zed_lateral/zed_lateral/imu/data", Imu, process_data)
    #sub3 = rospy.Subscriber("/zed_front/zed_node_front/imu/data", Imu, process_data)

    rospy.loginfo('test node has been started...')
    rospy.spin() #blocks until node is shutdown, Yields activity on other threads