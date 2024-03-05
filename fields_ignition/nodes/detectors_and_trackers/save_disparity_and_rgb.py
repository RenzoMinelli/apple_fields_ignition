#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
import message_filters
from rgb_and_depth_processing import track_filter_and_count
import os
import subprocess
import sys

def read_cameras():
    ros_namespace = os.getenv('ROS_NAMESPACE') == None and 'stereo' or os.getenv('ROS_NAMESPACE')
    imageL = message_filters.Subscriber("/" + ros_namespace + "/left/image_rect_color", Image)
    imageR = message_filters.Subscriber("/" + ros_namespace + "/right/image_rect_color", Image)
    disparity = message_filters.Subscriber("/" + ros_namespace + "/disparity", DisparityImage)

    # Synchronize images
    ts = message_filters.TimeSynchronizer([imageL, imageR, disparity], queue_size=20)
    ts.registerCallback(image_callback)
    rospy.spin()

def image_callback(imageL, imageR, disparity):
    br = CvBridge()
    rospy.loginfo("receiving Image")

    # convert the images to cv2 format and save them
    cv_image_left = br.imgmsg_to_cv2(imageL, 'bgr8')
    cv_image_right = br.imgmsg_to_cv2(imageR, 'bgr8')
    cv_disparity = br.imgmsg_to_cv2(disparity.image)

    timestamp = str(imageR.header.stamp)

    cv.imwrite('left_rgb_images/{}.png'.format(timestamp), cv_image_left)
    cv.imwrite('right_rgb_images/{}.png'.format(timestamp), cv_image_right)
    cv.imwrite('disparity_images/{}.png'.format(timestamp), cv_disparity)

def empty_folder(folder_path):
    # If the folder does not exist, create it
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
        return
    
    # Get all the file names in the folder
    file_names = os.listdir(folder_path)

    # Iterate over the file names and delete each file
    for file_name in file_names:
        file_path = os.path.join(folder_path, file_name)
        os.remove(file_path)

def delete_folder(folder_path):
    subprocess.run(['rm', '-rf', folder_path])

if __name__ == '__main__':
    rospy.init_node('save_disparity_and_rgb')

    working_directory = sys.argv[1]
    print("working inside directory ", working_directory)

    try:
        # Change the current directory to the one sent as argument
        os.chdir(working_directory)
        # Empty the folders
        empty_folder('left_rgb_images')
        empty_folder('right_rgb_images')
        empty_folder('disparity_images')
        delete_folder('yolo_tracking/runs/track/exp')

        read_cameras()

        # Process generated images
        rospy.on_shutdown(track_filter_and_count(working_directory))

        track_filter_and_count(working_directory)
    except rospy.ROSInterruptException:
        pass


# Bag de mercedes:
# en fields_ignition/launch/stereo.launch cambiar el path del bag en la linea 3 en args
# roslaunch fields_ignition stereo.launch
# ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc
# Si queres mejor calidad: rosrun rqt_reconfigure rqt_reconfigure y cargar el yaml en /home/paolo/catkin_ws/src/apple_fields_ignition/fields_ignition/config/stereoConfigMercedez.yaml
# rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color (para verificacion visual de que se ve bien)
# rosrun fields_ignition save_disparity_and_rgb.py

# Simulacion:
# cambiar en este py lo que indica el comentario en la linea 12
# roslaunch fields_ignition fields_ignition.launch
# ROS_NAMESPACE=costar_husky_sensor_config_1 rosrun stereo_image_proc stereo_image_proc
# Si queres mejor calidad: rosrun rqt_reconfigure rqt_reconfigure y cargar el yaml en /home/paolo/catkin_ws/src/apple_fields_ignition/fields_ignition/config/stereoConfigSim.yaml
# rosrun image_view stereo_view stereo:=/costar_husky_sensor_config_1 image:=image_rect_color (para verificacion visual de que se ve bien)
# rosrun fields_ignition save_disparity_and_rgb.py
