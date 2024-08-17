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
last_msg_time = None

def read_cameras():
    ros_namespace = os.getenv('ROS_NAMESPACE') if os.getenv('ROS_NAMESPACE') else 'stereo'
    imageL = message_filters.Subscriber("/" + ros_namespace + "/left/image_rect_color", Image)
    disparity = message_filters.Subscriber("/" + ros_namespace + "/disparity", DisparityImage)

    # Use ApproximateTimeSynchronizer instead of TimeSynchronizer
    ts = message_filters.TimeSynchronizer([imageL, disparity], queue_size=20)
    ts.registerCallback(image_callback)

    rospy.Timer(rospy.Duration(5), check_last_message)  # Check every 5 seconds

def map_distance_for_image(depth_map):
    return np.interp(depth_map, (1, 4), (0, 255))

T1 = None
MIN_PROCESSING_RATE = None
def image_callback(imageL, disparity):
    global T1
    global MIN_PROCESSING_RATE
    if not T1 is None:
        time_diff = (rospy.Time.now() - T1).to_sec() * 1000
        print("Tiempo entre imagenes: ", time_diff, "ms")
        processing_rate = 1000/time_diff
        print("se esta procesando a un rate de: ", processing_rate, "fps")
        if MIN_PROCESSING_RATE and processing_rate < MIN_PROCESSING_RATE:
            bag_max_running_rate = round(processing_rate/30, 3)
            print(f"ADVERTENCIA: para que el sistema pueda procesar el porcentaje minimo \
de fotogramas, actualmente: {MIN_PROCESSING_RATE} se debe correr \
el bag con la flag '--rate {bag_max_running_rate}'")
            
    T1 = rospy.Time.now()

    global last_msg_time
    last_msg_time = rospy.Time.now()

    br = CvBridge()
    rospy.loginfo("receiving Image")
    print("receiving Image")
    # convert the images to cv2 format
    cv_image_left = br.imgmsg_to_cv2(imageL, 'bgr8')
    cv_disparity = br.imgmsg_to_cv2(disparity.image)
    
    # Retrieve camera parameters
    focal_length = disparity.f  # Focal length
    baseline = disparity.T  # Baseline

    # Compute the depth map
    with np.errstate(divide='ignore', invalid='ignore'):  # Ignore division errors and invalid values
        depth_map = (focal_length * baseline) / cv_disparity
        
    depth_map = np.where(np.isinf(depth_map), 255, depth_map)
    timestamp = str(imageL.header.stamp)

    global FILTRO

    if FILTRO:
        FILTRO.track_filter_and_count_one_frame(timestamp, cv_image_left, depth_map)
        print(f"conteo por ahora: {FILTRO.get_apple_count()}")
    else: # solo generar para post procesado
        # Normalize depth map to range 0-255 for visualization
        depth_map_normalized = map_distance_for_image(depth_map)
      
        cv.imwrite('left_rgb_images/{}.png'.format(timestamp), cv_image_left)
        cv.imwrite('depth_maps/{}.png'.format(timestamp), depth_map_normalized)
        np.save(f"depth_matrix/{timestamp}.npy", depth_map)

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

def check_last_message(event):
    global last_msg_time
    if last_msg_time and (rospy.Time.now() - last_msg_time).to_sec() > 3:  # 3 seconds without messages
        rospy.loginfo("No messages received for 10 seconds, shutting down.")
        rospy.signal_shutdown("Finished processing bag")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--working_directory", required=True)
    parser.add_argument("--post_procesamiento", default='True', type=lambda x: (str(x).lower() == 'true'))
    parser.add_argument("--config", type=str)
    parser.add_argument("__name", type=str)
    parser.add_argument("__log", type=str)

    args = parser.parse_args()

    rospy.init_node('save_disparity_and_rgb')

    working_directory = args.working_directory
    print("working inside directory ", working_directory)

    if not args.post_procesamiento:
        metodo = args.metodo
        if metodo is None:
            raise Exception("No se ha especificado un metodo de post procesamiento")
        
        # need to delete foler runs/track/exp to avoid errors
        delete_folder(f"{os.path.dirname(os.path.realpath(__file__))}/yolo_tracking/runs/track/exp")

        from track_and_filter import TrackAndFilter
        FILTRO = TrackAndFilter(args.config)

    try:
        config = configparser.ConfigParser()
        config.read(args.config)

        MIN_PROCESSING_RATE = config.getfloat('SAVE_DISPARITY_AND_RGB', 'porcentaje_minimo_de_fotogramas_a_procesar')*30

        # Change the current directory to the one sent as argument
        os.chdir(working_directory)
        # Empty the folders
        empty_folder('left_rgb_images')
        empty_folder('depth_maps')
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
