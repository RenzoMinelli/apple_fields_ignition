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
from sklearn.cluster import KMeans
import sys
import json 

ros_namespace = os.getenv('ROS_NAMESPACE')

image_height = 1024
image_width = 1024
offset_horizontal = 53

# clone repository with tracker and tracker evaluator - after running the node for the first time, run `pip install --upgrade sentry-sdk`
def clone_tracker_repo():
    subprocess.run(["git", "clone", "--recurse-submodules", "https://github.com/PaoloCappelli/yolo_tracking.git"])
    subprocess.run(["pip", "install", "boxmot"])


# global variables
TRACKING_METHOD = "deepocsort"
YOLO_WEIGHTS = "weights/yolov8_100.pt"
bridge = CvBridge()
YOLOv8_model = None
FIXED_THRESHOLD = False

def find_clusters(lista):
    """
    Encuentra el punto que divide una lista ordenada de enteros en dos clusters,
    minimizando la suma de las varianzas internas de los clusters.
    """
    lista.sort()
    print('lista', lista)
    lista = numpy.array(lista)
    
    data = lista.reshape(-1, 1)

    # Initialize KMeans model
    kmeans = KMeans(n_clusters=2)

    # Fit the model to the data
    kmeans.fit(data)

    # Get the cluster centers
    cluster_centers = kmeans.cluster_centers_
    print('cluster centers: ', '0:', cluster_centers[0][0], '1: ',cluster_centers[1][0])
    return cluster_centers[0][0] if cluster_centers[0][0] > cluster_centers[1][0] else cluster_centers[1][0]


# read bounding boxes from the bounding box file
def read_bounding_boxes():

    # Get all detections except the file right_rgb_images.txt

    # Specify dir path
    dir_path = 'yolo_tracking/runs/track/exp/labels/'

    # Use os.listdir to obtain the file names skipping ignored_file
    file_names = os.listdir(dir_path)

    # Define bounding_boxes as an empty dictionary 
    bounding_boxes = {}

    # Iterate over file names
    for file_name in file_names:
        # Obtain the timestamp out of the file name (example name: image_40.png)
        timestamp = file_name.split(".")[0]

        # Obtain bounding boxes from the file
        with open(os.path.join(dir_path, file_name), 'r') as bb_file:
            lines = bb_file.readlines()

            for line in lines:
                line_split = line.split(" ")

                # If the line has 6 elements, the bounding box has an id, otherwise it is 0
                print("line: ", line)

                if len(line_split) < 6: # this is to skip IDs = 0 which correspond to unconfirmed tracks 
                    continue

                bb_id = int(line_split[5][:-1])
                x,y = line_split[1:3]

                # Convert everything to float first
                x = float(x)
                y = float(y)

                # As the values are normalized we need to multiply them by the image size
                x = x * image_width # ESTO HARDCODEADO NO ME PARECE MUCHO PORQUE SI ALGUIEN EN EL FUTURO QUIERE CAMBIAR EL SENSOR SE COMPLICA REVISAR EL CODIGO, ME PARECE QUE DEBERIA SER UN PARAMETRO O UNA VARIABLE GLOABL MINIMO
                y = y * image_height

                # Convert everything to int
                x = int(x)
                y = int(y)

                # if (x + w/2 < 70 or x + w/2 > image_width - 7 or y + h/2 < 7 or y + h/2 > image_height - 7): #ESTE IF ES PARA CONSIDERAR LA FRANJA NEGRA QUE SALE EN LAS IMAGENES DE PROFUNDIDAD
                #     continue

                if(int(x) + offset_horizontal >= image_width):
                    continue

                # Create a list with the bounding box center and the bounding box id which is what will be saved in the dictionary
                bb_center = [x + offset_horizontal, y, bb_id] #EL + 30 PARA CONSIDERAR LA FRANJA NEGRA QUE SALE EN LAS IMAGENES DEPROFUNDIDAD

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
    depth_image = cv2.imread("disparity_images/" + str(timestamp) + ".png", cv2.IMREAD_GRAYSCALE)

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
        if depth[1] >= threshold:
            filtered_depths.append(depth)
    return filtered_depths

def total_amount_apples():
    dir_path = "/home/renzo/catkin_ws/src/apple_fields_ignition/fields_ignition/generated/stereo_close_rows/apple_field/"
    apple_amount = 0
    for root, dirs, files in os.walk(dir_path):
        for dir_name in dirs:
            if dir_name.startswith('apple_'):
                data_file = os.path.join(root, dir_name, 'markers.json')
                if os.path.exists(data_file):
                    with open(data_file, 'r') as f:
                        data = json.load(f)
                        apple_amount += data["count_apples"]
    return apple_amount

# when the node is killed, run the tracker and filter the results
def track_filter_and_count(working_directory):
    os.chdir(working_directory)
    print('working inside directory ', os.getcwd())
    
    clone_tracker_repo()

    print('Running tracker and tracker evaluator...')
    SOURCE = "right_rgb_images"

    subprocess.run(["python3", "yolo_tracking/tracking/track.py", "--yolo-model", YOLO_WEIGHTS, "--tracking-method", TRACKING_METHOD, "--source", SOURCE, "--save", "--save-txt"]) 

    # get the bounding boxes from the file
    bounding_boxes = read_bounding_boxes()

    # get the depths of the bounding boxes
    depths = []
    
    for timestamp in bounding_boxes:
        depths.extend(get_depths(timestamp, bounding_boxes))

    # Filter the results using depth data
    # we must preserve those depths that are within [0, 30]. The rest must be filtered out
    threshold = 30 if FIXED_THRESHOLD else find_clusters([pair[1] for pair in depths]) 
    print('with calculated threshold: ', threshold)
    filtered_depths = filter_depths(depths, threshold)

    # Count the distinct ids that remained after filtering
    ids = []
    for depth in filtered_depths:
        ids.append(depth[0])
    ids = set(ids)
    # Print the number of apples
    print('Number of apples counted: ' + str(len(ids)))
    trees_counted = 5
    total_amount_trees = 15
    tot_apples = total_amount_apples()
    print(f"total_amount_apples: {tot_apples}, for_{trees_counted}_trees: {round(tot_apples/total_amount_trees)*trees_counted}")



if __name__ == "__main__":
    working_directory=sys.argv[1]
    track_filter_and_count(working_directory)