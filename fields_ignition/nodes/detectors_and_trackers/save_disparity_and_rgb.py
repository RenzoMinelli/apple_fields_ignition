#!/usr/bin/env python3
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
import message_filters
import os
import subprocess
import argparse
import numpy as np
import configparser

FILTRO = None
CAMERA_MODEL = None

def read_cameras():
    ros_namespace = os.getenv('ROS_NAMESPACE') if os.getenv('ROS_NAMESPACE') else MODEL
    if CAMERA_MODEL == 'depth':
        image_topic = "/medio/image_raw"
        depth_topic = "/medio/depth_image"
        depth_type = Image
    else:
        image_topic = "/left/image_rect_color"
        depth_topic = "/disparity"
        depth_type = DisparityImage

    image = message_filters.Subscriber("/" + ros_namespace + image_topic, Image)
    depth_data = message_filters.Subscriber("/" + ros_namespace + depth_topic, depth_type)

    # Sincronizar la imagen y la profundidad
    ts = message_filters.TimeSynchronizer([image, depth_data], queue_size=20)
    ts.registerCallback(image_callback)

def map_distance_for_image(depth_map):
    return np.interp(depth_map, (1, 4), (0, 255))

def image_callback(image, depth_data):
    br = CvBridge()
    rospy.loginfo("receiving Image")

    # Convertir la imagen y la profundidad a formato de OpenCV
    cv_image = br.imgmsg_to_cv2(image, 'bgr8')
    if CAMERA_MODEL == 'depth':
      cv_depth_data = br.imgmsg_to_cv2(depth_data)
    else:
      cv_depth_data = br.imgmsg_to_cv2(depth_data.image)
        
    # Asignar 255 a los valores infinitos representados como 'inf'
    depth_map = np.where(np.isinf(cv_depth_data), 255, cv_depth_data)

    timestamp = str(image.header.stamp)

    global FILTRO

    if FILTRO:
        FILTRO.track_filter_and_count_one_frame(timestamp, cv_image, depth_map)
        print(f"conteo por ahora: {FILTRO.get_apple_count()}")
    else:
        # Guardar los datos para post procesado
        np.save(f"depth_maps/{timestamp}.npy", depth_map)

        # Guardar las imagenes para visualizacion
        normalised_depth = map_distance_for_image(depth_map)
        cv.imwrite(f"depth_maps_visualizable/{timestamp}.png", normalised_depth)
        cv.imwrite(f"rgb_images/{timestamp}.png", cv_image)

def empty_folder(folder_path):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
        return

    file_names = os.listdir(folder_path)
    for file_name in file_names:
        file_path = os.path.join(folder_path, file_name)
        os.remove(file_path)

def delete_folder(folder_path):
    subprocess.run(['rm', '-rf', folder_path])

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--working_directory", required=True)
    parser.add_argument("--post_procesamiento", default='True', type=lambda x: (str(x).lower() == 'true'))
    parser.add_argument("--camera_model", type=str)
    parser.add_argument("--config", type=str)
    parser.add_argument("__name", type=str)
    parser.add_argument("__log", type=str)

    args = parser.parse_args()

    CAMERA_MODEL = args.camera_model
    if CAMERA_MODEL == 'stereo':
        print('Using stereo camera model, the saved rgb images are the ones from the left camera')
    elif CAMERA_MODEL:
        print('Using depth camera model')

    rospy.init_node('save_disparity_and_rgb')

    working_directory = args.working_directory
    print("working inside directory ", working_directory)

    if not args.post_procesamiento:
        metodo = args.metodo
        if metodo is None:
            raise Exception("No se ha especificado un metodo de post procesamiento")
        
        # Borrar la carpeta exp previa para evitar errores
        delete_folder(f"{os.path.dirname(os.path.realpath(__file__))}/yolo_tracking/runs/track/exp")

        from track_and_filter import TrackAndFilter
        FILTRO = TrackAndFilter(args.config)

    try:
        # Cambiar el directorio actual al que se envio como argumento
        os.chdir(working_directory)
        # Vaciar las carpetas
        empty_folder('rgb_images')
        empty_folder('depth_maps')
        empty_folder('depth_maps_visualizable')
        empty_folder('depth_matrix')
        delete_folder('yolo_tracking/runs/track/exp')

        
        read_cameras()
        rospy.spin()

        if FILTRO:
            print(f"Conteo final: {FILTRO.get_apple_count()}")

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
