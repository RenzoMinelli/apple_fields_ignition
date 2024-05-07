#!/usr/bin/env python3

# imports
import time as Time
# from ultralytics import YOLO
import cv2
# from cv_bridge import CvBridge, CvBridgeError
# import torch
import datetime
# import pdb
import subprocess
import os
import numpy
from sklearn.cluster import KMeans
import sys

ros_namespace = os.getenv('ROS_NAMESPACE')

image_height = 1024
image_width = 1024
offset_horizontal = 53

# global variables
YOLOv8_model = None
FIXED_THRESHOLD = False

def find_clusters(lista):
    """
    Encuentra el punto que divide una lista ordenada de enteros en dos clusters,
    minimizando la suma de las varianzas internas de los clusters.
    """
    lista.sort()
    lista = numpy.array(lista)
    
    data = lista.reshape(-1, 1)

    # Initialize KMeans model for 1 cluster
    kmeans_1 = KMeans(n_clusters=1, n_init=10)
    kmeans_1.fit(data)
    inertia_1 = kmeans_1.inertia_

    # Initialize KMeans model for 2 clusters
    kmeans_2 = KMeans(n_clusters=2, n_init=10)
    kmeans_2.fit(data)
    inertia_2 = kmeans_2.inertia_

    if inertia_1 < inertia_2:
        # print("Mejor con 1 cluster.")
        cluster_centers = kmeans_1.cluster_centers_
        print(f"cluster center: {cluster_centers[0][0]}")
        return cluster_centers[0][0]
    else:
        # print("Mejor con 2 clusters.")
        # Get the cluster centers for 2 clusters
        cluster_centers = kmeans_2.cluster_centers_
        print('cluster centers: ', '0:', cluster_centers[0][0], '1: ', cluster_centers[1][0])
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
                x,y,w,h = line_split[1:5]

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
                bb_center = [x + offset_horizontal, y, bb_id, w, h] #EL + 30 PARA CONSIDERAR LA FRANJA NEGRA QUE SALE EN LAS IMAGENES DEPROFUNDIDAD

                if (timestamp in bounding_boxes):
                    bounding_boxes[timestamp].append(bb_center)
                else:
                    bounding_boxes[timestamp] = [bb_center]

        

    # The return value is a dictionary with the timestamp as the key and an array with the bounding boxes centers of the corresponding frame as the value, and as a third value, the bounding box id.
    # bounding_boxes = {<timestamp>: [[<x>, <y>, <id>, <w>, <h>], ...], ...}
    return bounding_boxes

# when the nodes ends track the apples and evaluate the tracking

# given a sequence number (seq and a dictionary with the bounding boxes (bounding_boxes), return an array with the depths of the bounding boxes
def get_depths(timestamp, bounding_boxes):
    # read the depth image
    # print("warning: reading depth image from file using grayscale parameter. For other kinds of images, change the code.")
    depth_image = cv2.imread("disparity_images/" + str(timestamp) + ".png", cv2.IMREAD_GRAYSCALE)
    img = cv2.imread("disparity_images/" + str(timestamp) + ".png")

    if type(depth_image) != numpy.ndarray:
        print('timestamp ' + str(timestamp) + ' not found in detected_images_depth_data folder')
        return []

    # get the depths of the bounding boxes
    depths = []
    for bb_center in bounding_boxes[timestamp]:
        bb_id = bb_center[2]
        depth = depth_image[int(bb_center[1]), int(bb_center[0])]
        cv2.circle(img, (int(bb_center[0]), int(bb_center[1])), 5, (0, 0, 255), -1)

        depths.append([bb_id, depth, bb_center[0], bb_center[1], bb_center[3], bb_center[4]]) # OJO ESTO PUEDE ESTAR MAL
    cv2.imwrite('test_depth_i10/' + timestamp + '.png', img)

    # returns an array with the depths of the bounding boxes
    # depths = [[<id>, <depth>, <x>, <y>, <w>, <h>], ...]
    return depths

def filter_depths(depths, threshold):
    green_depths = []
    red_depths = []
    for depth in depths:
        if depth[1] >= threshold:
            green_depths.append(depth)
        else:
            red_depths.append(depth)

    return [red_depths, green_depths]


# Define a function to draw bounding boxes on an image and save the modified image
def draw_boxes_and_save(image_path, green_bboxs, red_bboxs, output_folder):
    # Read the image
    img = cv2.imread(image_path)

    # Iterate over each GREEN bounding box in the list
    for i, bbox in enumerate(green_bboxs):

        _, _, x, y, width, height = bbox

        w = float(width)*image_width
        h = float(height)*image_height

        x = x - offset_horizontal - w/2
        y -= h/2

        # Draw the bounding box
        cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)

    # Iterate over each RED bounding box in the list
    for i, bbox in enumerate(red_bboxs):

        _, _, x, y, width, height = bbox

        w = float(width)*image_width
        h = float(height)*image_height

        x = x - offset_horizontal - w/2
        y -= h/2

        # Draw the bounding box
        cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), (0, 0, 255), 2)

    # Save the modified image with bounding boxes
    filename = os.path.basename(image_path)
    output_path = os.path.join(output_folder, filename)

    # debug log
    print('image: ', image_path)
    print('saving image with bounding boxes in ', output_path)

    cv2.imwrite(output_path, img)


# when the node is killed, run the tracker and filter the results
def track_filter_and_count(working_directory):
    os.chdir(working_directory)
    print('working inside directory ', os.getcwd())
    
    # get the bounding boxes from the file
    # dictionary with the timestamp as the key and an array with the bounding boxes centers of the corresponding frame as the value, and as a third value, the bounding box id.
    # bounding_boxes = {<timestamp>: [[<x>, <y>, <id>, <w>, <h>], ...], ...}
    bounding_boxes = read_bounding_boxes()

    # get the depths of the bounding boxes
    # depths = []
    
    # for timestamp in bounding_boxes:
    #     depths.extend(get_depths(timestamp, bounding_boxes))

    # # Filter the results using depth data
    # # we must preserve those depths that are within [0, 30]. The rest must be filtered out
    # threshold = 30 if FIXED_THRESHOLD else find_clusters([pair[1] for pair in depths]) 
    
    for timestamp in bounding_boxes:

        # depths = [[<id>, <depth>, <x>, <y>, <w>, <h>], ...]
        image_depth_data = get_depths(timestamp, bounding_boxes)
        
        # Filter the results using depth data
        # we must preserve those depths that are within [0, 30]. The rest must be filtered out
        
        threshold = 30 if FIXED_THRESHOLD else find_clusters([pair[1] for pair in image_depth_data]) 

        print(f"--- threshold found: {threshold}")
        
        # red_depths, green_depths = [[<id>, <depth>, <x>, <y>, <w>, <h>], ...], [[<id>, <depth>, <x>, <y>, <w>, <h>], ...]
        # print <timestamp>.png with the bounding boxes of the filtered apples colored in green and the rest in red
        red_depths, green_depths = filter_depths(image_depth_data, threshold)
        

        image_path = 'right_rgb_images/' + timestamp + '.png'
        output_folder = working_directory + '/test_filtered_images'
        draw_boxes_and_save(image_path, green_depths, red_depths, output_folder)


if __name__ == "__main__":
    working_directory=sys.argv[1]

    folder_names = ["test_depth_i10", "test_filtered_images"]
    for folder_name in folder_names:
        folder_path = f"{working_directory}/{folder_name}"
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        else:
            files = os.listdir(folder_path)
            for file_name in files:
                file_path = os.path.join(folder_path, file_name)
                os.remove(file_path)

    track_filter_and_count(working_directory)