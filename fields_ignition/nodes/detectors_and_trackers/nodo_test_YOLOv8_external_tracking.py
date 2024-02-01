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
import numpy

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
FIXED_THRESHOLD = True

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

    # Get all detections except the file detected_images_YOLOv8.txt

    # Specify dir path
    dir_path = 'yolo_tracking/runs/track/exp/labels/'
    ignored_file = 'detected_images_YOLOv8.txt'

    # Use os.listdir to obtain the file names skipping ignored_file
    
    file_names = os.listdir(dir_path)
    file_names.remove(ignored_file)

    # Define bounding_boxes as an empty dictionary 
    bounding_boxes = {}

    # Iterate over file names
    for file_name in file_names:
        # Obtain the timestamp out of the file name (example name: image_40.png)
        timestamp = file_name.split("_")[1].split(".")[0]

        # Obtain bounding boxes from the file
        with open(os.path.join(dir_path, file_name), 'r') as bb_file:
            lines = bb_file.readlines()

            for line in lines:
                line_split = line.split(" ")

                # If the line has 6 elements, the bounding box has an id, otherwise it is 0
                bb_id = int(line_split[5][:-1]) if len(line_split) == 6 else 0
                x,y = line_split[1:3]
                h,w = line_split[3:5]

                # Convert everything to float first
                x = float(x)
                y = float(y)
                h = float(h)
                w = float(w)

                # As the values are normalized we need to multiply them by the image size
                x = x * 1280 # ESTO HARDCODEADO NO ME PARECE MUCHO PORQUE SI ALGUIEN EN EL FUTURO QUIERE CAMBIAR EL SENSOR SE COMPLICA REVISAR EL CODIGO, ME PARECE QUE DEBERIA SER UN PARAMETRO O UNA VARIABLE GLOABL MINIMO
                y = y * 720
                h = h * 1280
                w = w * 720

                # Convert everything to int
                x = int(x)
                y = int(y)
                h = int(h)
                w = int(w)

                # Create a list with the bounding box center and the bounding box id which is what will be saved in the dictionary
                bb_center = [int(x + w/2), int(y + h/2), bb_id]

                if (timestamp in bounding_boxes):
                    bounding_boxes[timestamp].append(bb_center)
                else:
                    bounding_boxes[timestamp] = [bb_center]

    # The return value is a dictionary with the timestamp as the key and an array with the bounding boxes centers of the corresponding frame as the value, and as a third value, the bounding box id.
    return bounding_boxes

# when the nodes ends track the apples and evaluate the tracking

# given a sequence number (seq and a dictionary with the bounding boxes (bounding_boxes), return an array with the depths of the bounding boxes
def get_depths(timestamp, bounding_boxes):
    # read the depth image
    print("warning: reading depth image from file using grayscale parameter. For other kinds of images, change the code.")
    depth_image = cv2.imread("detected_images_depth_data/" + "image_" +str(timestamp) + ".png", cv2.IMREAD_GRAYSCALE)

    if type(depth_image) != numpy.ndarray:
        print('timestamp ' + str(timestamp) + ' not found in detected_images_depth_data folder')
        return []

    # get the depths of the bounding boxes
    depths = []
    for bb_center in bounding_boxes[timestamp]:
        bb_id = bb_center[2]
        depth = depth_image[int(bb_center[1]), int(bb_center[0])]

        depths.append([bb_id, depth])
    return depths

def filter_depths(depths, threshold):
    filtered_depths = []
    for depth in depths:
        if depth[1] <= threshold:
            filtered_depths.append(depth)
    return filtered_depths

# when the node is killed, run the tracker and filter the results
def track_filter_and_count():
    print('Running tracker and tracker evaluator...')
    
    SOURCE = "detected_images_YOLOv8"

    # subprocess.run(["python3", "yolo_tracking/examples/track.py", "--yolo-model", YOLO_WEIGHTS, "--tracking-method", TRACKING_METHOD, "--source", SOURCE, "--save", "--save-txt"]) 

    # get the bounding boxes from the file
    bounding_boxes = read_bounding_boxes()

    # get the depths of the bounding boxes
    depths = []
    
    for timestamp in bounding_boxes:
        depths.extend(get_depths(timestamp, bounding_boxes))

    # Filter the results using depth data
    # we must preserve those depths that are within [0, 30]. The rest must be filtered out
    threshold = 30 if FIXED_THRESHOLD else encontrar_punto_de_corte([pair[1] for pair in depths]) 
    filtered_depths = filter_depths(depths, threshold)

    # Count the distinct ids that remained after filtering
    ids = []
    for depth in filtered_depths:
        ids.append(depth[0])
    ids = list(set(ids))
    # Print the number of apples
    print('Number of apples: ' + str(len(ids)))

# saves an image and returns its name
def save_image(save_path, saved_image_name, global_frame):
    current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S%f") + "_"
    cv2.imwrite(save_path + '/' + current_time + saved_image_name, global_frame)
    return current_time + saved_image_name

# callback function
def process_data(data):
    try:
        pdb.set_trace()
        global_frame = bridge.compressed_imgmsg_to_cv2(data)
        timestamp = data.header.stamp

        # save image named as timestamp in order to match with the depth data
        save_image("detected_images_YOLOv8", str(timestamp) + ".jpg", global_frame)

    except CvBridgeError as e:
        raise(e)


# main function
if __name__ == '__main__':

    rospy.init_node('tracking_and_filtering_node')

    rospy.loginfo('tracking and filtering node started')
   
    # TODO: WAIT FOR DEPTH NODE TO FINISH AND PROCESS GENERATED DATA
    track_filter_and_count()
    