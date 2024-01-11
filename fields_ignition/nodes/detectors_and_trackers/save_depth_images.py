#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import message_filters

def read_cameras():
    imageL = message_filters.Subscriber("/costar_husky_sensor_config_1/left/image_raw/compressed", CompressedImage)
    imageR = message_filters.Subscriber("/costar_husky_sensor_config_1/right/image_raw/compressed", CompressedImage)
    

    # Synchronize images
    ts = message_filters.ApproximateTimeSynchronizer([imageL, imageR], queue_size=10, slop=0.5)
    ts.registerCallback(image_callback)
    rospy.spin()

def image_callback(imageL, imageR):
    br = CvBridge()
    rospy.loginfo("receiving Image")

    imageLeft = br.compressed_imgmsg_to_cv2(imageL, "rgb8")
    imageRight = br.compressed_imgmsg_to_cv2(imageR, "rgb8")

    imageL_new=cv.cvtColor(imageLeft, cv.COLOR_BGR2GRAY)

    imageR_new=cv.cvtColor(imageRight, cv.COLOR_BGR2GRAY)

   

    stereo = cv.StereoBM_create(numDisparities=16, blockSize=15)
    disparity = stereo.compute(imageR_new,imageL_new)






    rospy.loginfo("showing depth image")
    # Process images...

    pub = rospy.Publisher('/left_camera', Image, queue_size=1)
    cv.imshow('disparity', disparity)
    msg = br.cv2_to_imgmsg(disparity, encoding='16SC1')
    pub2 = rospy.Publisher('/Right_camera', Image, queue_size=1)
    msg2 = br.cv2_to_imgmsg(disparity, encoding='16SC1')

    
    pub.publish(msg)
    pub2.publish(msg2)
     

    

if __name__ == '__main__':
    rospy.init_node('my_node')
    try:
        read_cameras()
        
    except rospy.ROSInterruptException:
        pass
