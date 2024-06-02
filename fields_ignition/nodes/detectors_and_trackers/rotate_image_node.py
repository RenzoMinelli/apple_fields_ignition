#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageRotator:
    def __init__(self):
        self.bridge = CvBridge()
        self.left_image_sub = rospy.Subscriber("/zed_lateral/zed_lateral/left/image_rect_color/compressed", CompressedImage, self.left_callback)
        self.right_image_sub = rospy.Subscriber("/zed_lateral/zed_lateral/left/image_rect_color/compressed", CompressedImage, self.right_callback)
        self.left_image_pub = rospy.Publisher("/stereo/left/image_raw/compressed", CompressedImage, queue_size=1)
        self.right_image_pub = rospy.Publisher("/stereo/right/image_raw/compressed", CompressedImage, queue_size=1)

    def left_callback(self, data):
        try:
            # Descomprimir la imagen
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Rotar la imagen
            rotated_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
            
            # Comprimir la imagen nuevamente
            compressed_image = cv2.imencode('.jpg', rotated_image)[1].tostring()
            
            # Crear mensaje CompressedImage y publicar
            image_message = CompressedImage()
            image_message.header = data.header
            image_message.format = "jpeg"
            image_message.data = compressed_image
            self.left_image_pub.publish(image_message)
        except CvBridgeError as e:
            rospy.logerr(e)

    def right_callback(self, data):
        try:
            # Descomprimir la imagen
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Rotar la imagen
            rotated_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
            
            # Comprimir la imagen nuevamente
            compressed_image = cv2.imencode('.jpg', rotated_image)[1].tostring()
            
            # Crear mensaje CompressedImage y publicar
            image_message = CompressedImage()
            image_message.header = data.header
            image_message.format = "jpeg"
            image_message.data = compressed_image
            self.right_image_pub.publish(image_message)
        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('image_rotator', anonymous=True)
    ImageRotator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
