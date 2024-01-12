#!/usr/bin/env python3

# imports
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
import time as Time
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import datetime
import pdb
import subprocess
import os

# clone repository with tracker and tracker evaluator - after running the node for the first time, run `pip install --upgrade sentry-sdk`
subprocess.run(["git", "clone", "--recurse-submodules", "https://github.com/Geromendez135/yolo_tracking.git"])
subprocess.run(["pip", "install", "-r", "yolo_tracking/requirements.txt"])
subprocess.run(["git", "clone", "https://github.com/JonathonLuiten/TrackEval.git", "yolo_tracking/val_utils"])
subprocess.run(["cp", "-r", "yolo_tracking/boxmot", "yolo_tracking/examples"])


# global variables
TRACKING_METHOD = "bytetrack"
YOLO_WEIGHTS = "weights/yolov8.pt"
bridge = CvBridge()
YOLOv8_model = None

def encontrar_punto_de_corte(lista):
    """
    Encuentra el punto que divide una lista ordenada de enteros en dos clusters,
    minimizando la suma de las varianzas internas de los clusters.
    """
    # Ordenar la lista
    lista_ordenada = sorted(lista)

    # Inicializar el mejor punto de corte y la menor suma de varianzas encontrada
    mejor_punto = None
    menor_varianza_suma = float('inf')

    # Iterar sobre todos los puntos posibles (excepto los extremos)
    for i in range(1, len(lista_ordenada)):
        # Dividir la lista en dos clusters basándonos en el punto actual
        cluster_1 = lista_ordenada[:i]
        cluster_2 = lista_ordenada[i:]

        # Calcular la varianza de cada cluster
        varianza_1 = varianza(cluster_1) if cluster_1 else 0
        varianza_2 = varianza(cluster_2) if cluster_2 else 0

        # Sumar las varianzas
        varianza_suma = varianza_1 + varianza_2

        # Actualizar el mejor punto y la menor varianza suma si es necesario
        if varianza_suma < menor_varianza_suma:
            menor_varianza_suma = varianza_suma
            mejor_punto = lista_ordenada[i - 1]

    return mejor_punto

def varianza(lista):
    """
    Calcula la varianza de una lista de números.
    """
    media = sum(lista) / len(lista)
    varianza = sum((x - media) ** 2 for x in lista) / len(lista)
    return varianza

# read bounding boxes from the bounding box file
def read_bounding_boxes():
    # read the bounding boxes from the file
    with open("yolo_tracking/runs/track/exp/labels/detected_images_YOLOv8.txt", "r") as bb_file:
        lines = bb_file.readlines()
        bounding_boxes = []
        for line in lines:
            x,y = line.split(" ")[2:4]
            h,w = line.split(" ")[4:6]
            bounding_boxes.append([int(x) + int(w)/2, int(y) + int(h)/2])
    return bounding_boxes

# when the nodes ends track the apples and evaluate the tracking
def exit_handler():
    print('Running tracker and tracker evaluator...')
    
    SOURCE = "detected_images_YOLOv8"

    # if for some reason you want to run the tracker separately, run the following command in the terminal: (make sure you type in the correct arguments)
    # python3 yolov8_tracking/examples/track.py --yolo-model weights_yolov8l.pt --tracking-method bytetrack --source detected_images_YOLOv8 --save --hide-label --hide-conf
    subprocess.run(["python3", "yolo_tracking/examples/track.py", "--yolo-model", YOLO_WEIGHTS, "--tracking-method", TRACKING_METHOD, "--source", SOURCE, "--save", "--save-txt"]) 

    # filter the results using depth data

# saves an image and returns its name
def save_image(save_path, saved_image_name, global_frame):
    current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S%f") + "_"
    cv2.imwrite(save_path + '/' + current_time + saved_image_name, global_frame)
    return current_time + saved_image_name

# callback function
def process_data(data):
    try:
        global_frame = bridge.compressed_imgmsg_to_cv2(data)
        # save image with no annotations first
        saved_image_name = save_image("detected_images_YOLOv8", "detected.jpg", global_frame)

        # pdb.set_trace()
    except CvBridgeError as e:
        raise(e)


# main function
if __name__ == '__main__':

    bridge = CvBridge()

    rospy.init_node('test_node')
    
    # read from simulation
    sub = rospy.Subscriber("/costar_husky_sensor_config_1/left/image_raw/compressed", CompressedImage, process_data, queue_size = 10) 

    # read from bag
    # sub = rospy.Subscriber("/zed_lateral/zed_lateral/left/image_rect_color/compressed", CompressedImage, process_data, queue_size = 10) 

    rospy.loginfo('test node has been started...')

    # when the node is killed, run the tracker and the tracker evaluator
    rospy.on_shutdown(exit_handler)
    
    rospy.spin() #blocks until node is shutdown, Yields activity on other threads