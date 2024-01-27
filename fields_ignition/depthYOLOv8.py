#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import PointCloud2
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
        color_frame = bridge.compressed_imgmsg_to_cv2(color_data)

        # save the image with the sequence number of color_data
        cv2.imwrite(f"image_{color_data.header.seq}.jpg", color_frame)

        breakpoint()


    except CvBridgeError as e:
        rospy.logerr(f"Error converting images: {e}")

if __name__ == '__main__':
    bridge = CvBridge()
    # YOLOv8_model = YOLO('./src/apple_fields_ignition/weights.pt')

    rospy.init_node('apple_detection_node')

    color_sub = Subscriber("/costar_husky_sensor_config_1/left/image_rect_color/compressed", CompressedImage)
    disparity_sub = Subscriber("/costar_husky_sensor_config_1/points2", PointCloud2)

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

# computar a mano las profundidades pero en el bag de mercedez a ver como se ve, el stereo image proc anda maso, pero al menos si se ven imagenes de pointcloud
# VER LINK FAVORITO DE COMO CONFIGURAR BIEN LA CAMARA OSEA LOS PARAMETROS
# investigar como poder guardar la imagen de profundidad generada por rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color
# probar de configurar bien la camara para ver si obtengo cosas decentes
# porbar la simulacion con el depth a ver que tipo tiene, capaz puedo bypassear algo ahi y despues cuando tenga lo otro bien lo hago con eso otro noc
        
# notas:
# la nuve de puntos no anda en la sim, calibrar la camara? se ve doble, ver como arreglar eso
# ya tengo los mejores params, pero como genero esas mismas imagenes que genera el stereo_view
