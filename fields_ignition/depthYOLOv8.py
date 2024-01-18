#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from stereo_msgs.msg import DisparityImage
from message_filters import TimeSynchronizer, Subscriber
import time as Time
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import datetime
import pdb
import os

# Constants
UMBRAL = 50  # Adjust this threshold as needed

def process_data(color_data, disparity_data):
    try:
        # Convert color image
        # print the current directory
        # print(os.getcwd())
        color_frame = bridge.compressed_imgmsg_to_cv2(color_data)

        # Convert disparity image
        disparity_image = bridge.imgmsg_to_cv2(disparity_data.image, desired_encoding="passthrough")

        # Process color frame and disparity image to detect apples
        results = YOLOv8_model.track(color_frame, persist=True)

        # Filter detections based on disparity values at the center of the boxes
        filtered_results = []

        for box in results[0].boxes.xywh:
            # pdb.set_trace()
            center_x = int(box[0])
            center_y = int(box[1])

            # Get disparity value at the center pixel
            disparity_value = disparity_image[center_y, center_x]

            # if disparity_value > UMBRAL:
                # remove the box from the results
                # results[0].boxes.remove(box.id)

        # if filtered_results:
        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Save the image
        current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S%f")
        cv2.imwrite("./src/apple_fields_ignition/detected_images_YOLOv8/" + current_time + "_detected.jpg", annotated_frame)

    except CvBridgeError as e:
        rospy.logerr(f"Error converting images: {e}")

if __name__ == '__main__':
    bridge = CvBridge()
    YOLOv8_model = YOLO('./src/apple_fields_ignition/weights.pt')

    rospy.init_node('apple_detection_node')

    color_sub = Subscriber("/costar_husky_sensor_config_1/left/image_rect_color/compressed", CompressedImage)
    disparity_sub = Subscriber("/costar_husky_sensor_config_1/disparity", DisparityImage)

    ts = TimeSynchronizer([color_sub, disparity_sub], 10)
    ts.registerCallback(process_data)

    rospy.loginfo('Apple detection node has been started...')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Apple detection node has been terminated.")

# run in different terminals inside the catkin_ws folder
# roscore
# rosbag play --clock ./src/apple_fields_ignition/fields_ignition/generated/sim_stereo/experiments/Nombre/record.bag
# ROS_NAMESPACE=costar_husky_sensor_config_1 rosrun stereo_image_proc stereo_image_proc
# rosrun fields_ignition depthYOLOv8.py

USAR LOS POINT CLOUD, LA DISPARITI ES NADA QUE VER, VER DE GUARDAR LA DATA DE LOS POINTCLOUD Y PASARLO A IMAGEN CAPAZ A VER COMO SE VE, O ACCEDER
A MANO A UNOS PIXELES A VER SI TENE SENTIDO NOC 

VER LINK FAVORITO DE COMO CONFIGURAR BIEN LA CAMARA OSEA LOS PARAMETROS
