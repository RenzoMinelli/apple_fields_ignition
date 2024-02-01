#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from stereo_msgs.msg import DisparityImage
import message_filters
from rgb_and_depth_processing import track_filter_and_count

def read_cameras():
    #si es la simulacion cambiar /stereo por /costar_husky_sensor_config_1, lo que diga el topic de salida de hacer stereo_image_proc
    imageR = message_filters.Subscriber("/costar_husky_sensor_config_1/right/image_rect_color", Image)
    disparity = message_filters.Subscriber("/costar_husky_sensor_config_1/disparity", DisparityImage)

    # Synchronize images
    ts = message_filters.TimeSynchronizer([imageR, disparity], queue_size=20)
    ts.registerCallback(image_callback)
    rospy.spin()

def image_callback(imageR, disparity):
    br = CvBridge()
    rospy.loginfo("receiving Image")

    # convert the images to cv2 format and save them
    cv_image_right = br.imgmsg_to_cv2(imageR, 'bgr8')
    cv_disparity = br.imgmsg_to_cv2(disparity.image)

    timestamp = str(imageR.header.stamp)

    print("saving images")
    cv.imwrite('detected_images_depth_data/{}.png'.format(timestamp), cv_disparity)
    cv.imwrite('detected_images_YOLOv8/{}.png'.format(timestamp), cv_image_right)

if __name__ == '__main__':
    rospy.init_node('my_node')
    try:
        read_cameras()

        # Process generated images
        rospy.on_shutdown(track_filter_and_count)
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
# cambiar en este py lo que inica el comentario en la linea 12
# roslaunch fields_ignition fields_ignition.launch
# ROS_NAMESPACE=costar_husky_sensor_config_1 rosrun stereo_image_proc stereo_image_proc
# rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color
# Si queres mejor calidad: rosrun rqt_reconfigure rqt_reconfigure y cargar el yaml en /home/paolo/catkin_ws/src/apple_fields_ignition/fields_ignition/config/stereoConfigSim.yaml
# rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color (para verificacion visual de que se ve bien)
# rosrun fields_ignition save_disparity_and_rgb.py
