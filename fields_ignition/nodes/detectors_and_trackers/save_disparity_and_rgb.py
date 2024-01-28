#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from stereo_msgs.msg import DisparityImage
import message_filters

def read_cameras():
    print('Running stereo_image_proc')

    # Si es la simulacion cambiar /stereo por /costar husky o algo asi, lo que diga el topic de salida de hacer stereo_image_proc
    imageR = message_filters.Subscriber("/stereo/right/image_rect_color", Image)
    disparity = message_filters.Subscriber("stereo/disparity", DisparityImage)
    

    # Synchronize images
    ts = message_filters.TimeSynchronizer([imageR, disparity], queue_size=20)
    ts.registerCallback(image_callback)
    rospy.spin()

def image_callback(imageR, disparity):
    br = CvBridge()
    rospy.loginfo("receiving Image")

    # convert the images to cv2 format and save them
    cv_image_right = br.imgmsg_to_cv2(imageR)
    cv_disparity = br.imgmsg_to_cv2(disparity.image)

    seq = imageR.header.seq

    breakpoint()
    # get the timestamps that will be the name of the images, this is so the tracker can match the detected images with the depth data
    cv_disparity_timestamp = disparity.header.stamp
    cv_image_right_timestamp = imageR.header.stamp

    print("saving images")
    cv.imwrite('detected_images_depth_data/{}.png'.format(cv_disparity_timestamp), cv_disparity)
    cv.imwrite('detected_images_YOLOv8/{}.png'.format(cv_image_right_timestamp), cv_image_right)






    # rospy.loginfo("showing depth image")
    # Process images...

    # pub = rospy.Publisher('/left_camera', Image, queue_size=1)
    # cv.imshow('disparity', disparity)
    # msg = br.cv2_to_imgmsg(disparity, encoding='16SC1')
    # pub2 = rospy.Publisher('/Right_camera', Image, queue_size=1)
    # msg2 = br.cv2_to_imgmsg(disparity, encoding='16SC1')
    # save the images in the folder detected_images_depth_data
    # seq = imageL.header.seq
    # cv.imwrite('detected_images_depth_data/disparity_{}.png'.format(seq), disparity)

    
    # pub.publish(msg)
    # pub2.publish(msg2)
     

    

if __name__ == '__main__':
    rospy.init_node('my_node')
    try:
        read_cameras()
        
    except rospy.ROSInterruptException:
        pass


# roslaunch fields_ignition stereo.launch
# ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc
# rosrun rqt_reconfigure rqt_reconfigure y lodear el yaml en /home/paolo/catkin_ws/src/apple_fields_ignition/fields_ignition/config/stereoConfigMercedez.yaml
# rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color para verificacion visual de que se ve bien]
# rosrun fields_ignition save_disparity_and_rgb.py
# no se xq por un momento se veian bien y despues se ven medias negras, tengo que pasarle los args del yaml a stereo_image_proc
