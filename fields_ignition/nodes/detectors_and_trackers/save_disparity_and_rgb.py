#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from stereo_msgs.msg import DisparityImage
import message_filters

def read_cameras():
    #si es la simulacion cambiar /stereo por /costar husky o algo asi, lo que diga el topic de salida de hacer stereo_image_proc
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

    print("saving images")
    cv.imwrite('detected_images_depth_data/image_{}.png'.format(seq), cv_disparity)
    cv.imwrite('detected_images_YOLOv8/image_{}.png'.format(seq), cv_image_right)






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


# rosrun fields_ignition save_disparity_and_rgb.py
# rosbag play /home/paolo/catkin_ws/s1_230228.bag
# ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc
