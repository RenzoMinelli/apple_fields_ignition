#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import message_filters

def read_cameras():
    imageL = message_filters.Subscriber("/zed_lateral/zed_lateral/left/image_rect_color/compressed", CompressedImage)
    imageR = message_filters.Subscriber("/zed_lateral/zed_lateral/right/image_rect_color/compressed", CompressedImage)
    

    # Synchronize images
    ts = message_filters.TimeSynchronizer([imageL, imageR], queue_size=20)
    ts.registerCallback(image_callback)
    rospy.spin()

def image_callback(imageL, imageR):
    br = CvBridge()
    rospy.loginfo("receiving Image")

    # print("Sequence numbers:")
    # print("     Left: ", imageL.header.seq)
    # print("    Right: ", imageR.header.seq)
    if imageL.header.seq != imageR.header.seq:
        rospy.logerr("Images are not synchronized")
        # print the sequence numbers of the images
        print("Left: ", imageL.header)
        print("Right: ", imageR.header)
        # return

    cv_image_left = br.compressed_imgmsg_to_cv2(imageL)
    cv_image_right = br.compressed_imgmsg_to_cv2(imageR)
    cv_image_left_new = cv.cvtColor(cv_image_left, cv.COLOR_BGR2GRAY)
    cv_image_right_new = cv.cvtColor(cv_image_right, cv.COLOR_BGR2GRAY)

    print("creating stereo image")
    stereo = cv.StereoBM_create(numDisparities=32, blockSize=13)
    depth = stereo.compute(cv_image_left_new, cv_image_right_new)
    # save the images in the folder detected_images_depth_data
    seq = imageL.header.seq
    cv.imwrite('detected_images_depth_data/depth_{}.png'.format(seq), depth)
    cv.imwrite('detected_images_YOLOv8/left_{}.png'.format(seq), cv_image_left)






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


# rosrun fields_ignition save_depth_images.py
# rosbag play /home/paolo/catkin_ws/s1_230228.bag
