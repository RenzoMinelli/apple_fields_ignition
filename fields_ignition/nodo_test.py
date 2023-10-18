#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2

i = 0  # Initialize i

def convert_depth_image(ros_image):
    global i  # Declare 'i' as a global variable
    # print(ros_image.format)
    # print(ros_image.data[:10])
    cv_bridge = CvBridge()
    try:
        depth_image = cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
    except CvBridgeError as e:
        print(e)
        return

    

    depth_array = np.array(depth_image, dtype=np.float32)
    # np.save("/home/paolo/Desktop/pruebas/depth_img{}.npy".format(i), depth_array)
    rospy.loginfo(depth_array.shape)

    if i == 0:
        print(depth_array[434][905])
        print(depth_array[939][1077])
        print(depth_array[945][1090])
    # save the matrix as a txt file
    np.savetxt("/home/paolo/Desktop/pruebas/depth_img{}.txt".format(i), depth_array, delimiter=",")

    # # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    # cv2.imwrite("/home/paolo/Desktop/pruebas/depth_img{}.png".format(i), depth_colormap)

    i += 1  # Increment i
    # Or, if you don't want to use global variable 'i', you can pass it as an argument to the callback.

def pixel2depth():
    rospy.init_node('pixel2depth')
    rospy.Subscriber("/costar_husky_sensor_config_1/front/depth", Image, callback=convert_depth_image, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    pixel2depth()

