#!/usr/bin/env python3

# imports
from ultralytics import YOLO
import cv2
import subprocess
import os
import json 
from plane_processing_utils import filtrar_puntos, CantidadPuntosInsuficiente
import argparse

ros_namespace = os.getenv('ROS_NAMESPACE')

image_height = 1024
image_width = 1024

# clone repository with tracker and tracker evaluator - after running the node for the first time, run `pip install --upgrade sentry-sdk`
def clone_tracker_repo():
    subprocess.run(["git", "clone", "--recurse-submodules", "https://github.com/PaoloCappelli/yolo_tracking.git"])

    # in folder yolo_tracking run git pull to get the latest version
    os.chdir("yolo_tracking")
    
    subprocess.run(["git", "reset", "--hard","origin/master"])
    subprocess.run(["git", "pull"])

    # go back to the original directory and install the tracker
    os.chdir("..")
    subprocess.run(["pip", "install", "boxmot"])

# global variables
TRACKING_METHOD = "deepocsort"
YOLO_WEIGHTS = "weights/yolov8l_150.pt"
YOLOv8_model = None
FIXED_THRESHOLD = False
WORLD_NAME = "stereo_trees_close"
SOURCE = "left_rgb_images"

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

                # Create a list with the bounding box center and the bounding box id which is what will be saved in the dictionary
                bb_center = [x, y, bb_id] #EL + 30 PARA CONSIDERAR LA FRANJA NEGRA QUE SALE EN LAS IMAGENES DEPROFUNDIDAD

                if (timestamp in bounding_boxes):
                    bounding_boxes[timestamp].append(bb_center)
                else:
                    bounding_boxes[timestamp] = [bb_center]

    # The return value is a dictionary with the timestamp as the key and an array with the bounding boxes centers of the corresponding frame as the value, and as a third value, the bounding box id.
    return bounding_boxes

def total_amount_apples(working_directory):
    dir_path = f"{working_directory}/src/apple_fields_ignition/fields_ignition/generated/{WORLD_NAME}/apple_field/"
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

def total_amount_trees(working_directory):
    dir_path = f"{working_directory}/src/apple_fields_ignition/fields_ignition/generated/{WORLD_NAME}/apple_field/"
    tree_amount = 0
    for root, dirs, files in os.walk(dir_path):
        for dir_name in dirs:
            if dir_name.startswith('apple_'):
                tree_amount += 1
    return tree_amount

def total_amount_apples_for_trees_ids(working_directory, ids):
    dir_path = f"{working_directory}/src/apple_fields_ignition/fields_ignition/generated/{WORLD_NAME}/apple_field/"
    apple_amount = 0
    dir_names_wanted = [f"apple_{x}" for x in ids]
    for root, dirs, files in os.walk(dir_path):
        for dir_name in dir_names_wanted:
            data_file = os.path.join(root, dir_name, 'markers.json')
            if os.path.exists(data_file):
                with open(data_file, 'r') as f:
                    data = json.load(f)
                    apple_amount += data["count_apples"]
    return apple_amount

# run the tracker and filter the results
def track_filter_and_count(working_directory, track, generar_imagen_plano):
    os.chdir(working_directory)
    print('working inside directory ', os.getcwd())
    
    clone_tracker_repo()

    print('Running tracker and tracker evaluator...')

    if track:
        # run the tracker
        subprocess.run(["python3", "yolo_tracking/tracking/track.py", "--yolo-model", YOLO_WEIGHTS, "--tracking-method", TRACKING_METHOD, "--source", SOURCE, "--save", "--save-txt"]) 

    if generar_imagen_plano:
        folder_path = f"{working_directory}/planos"
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        else:
            # vaciar carpeta de planos
            files = os.listdir(folder_path)
            for file_name in files:
                file_path = os.path.join(folder_path, file_name)
                os.remove(file_path)

    # get the bounding boxes from the file
    bounding_boxes = read_bounding_boxes()

    # get the points of the bounding boxes
    points = []
    
    for timestamp in bounding_boxes:
        # print(f"TIMESTAMP: {timestamp}")
        # read original image to img_original
        img_original = cv2.imread("left_rgb_images/" + str(timestamp) + ".png")

        # read the depth image to mapa_profundidad
        mapa_profundidad = cv2.imread("disparity_images/" + str(timestamp) + ".png", cv2.IMREAD_GRAYSCALE)        
        
        # filter points using generated plane based on trunk detection and depth data
        filtered_points = []
        skipped_points = []

        try:
            filtered_points = filtrar_puntos(timestamp,bounding_boxes[timestamp], img_original, mapa_profundidad, working_directory, generar_imagen_plano)
        except CantidadPuntosInsuficiente as e:
            print(f"frame skipped, error: {e}")
            # print(traceback.format_exc())

        #print(f"puntos filtrados: {filtered_points}")
        #print(f"puntos rechazados: {skipped_points}")
        points.extend(filtered_points)

    # Count the distinct ids that remained after filtering
    ids = []
    for point in points:
        ids.append(point[2])
    ids = set(ids)
    # Print the number of apples
    print('Number of apples counted: ' + str(len(ids)))

    # trees_counted = 5
    # tot_trees = total_amount_trees(working_directory)
    # tot_apples = total_amount_apples(working_directory)
    # print(f"total_amount_apples: {tot_apples}, for_{trees_counted}_trees: {round((tot_apples*trees_counted)/tot_trees)}")
    print(f"amount of apples exactly for trees id (5,6,7,8,9): {total_amount_apples_for_trees_ids(working_directory, [5,6,7,8,9])}")

if __name__ == "__main__":
    # execution example
    # python3 /home/pincho/catkin_ws/src/apple_fields_ignition/fields_ignition/nodes/detectors_and_trackers/plane_method/plane_rgb_and_depth_processing.py --working_directory '/home/pincho/catkin_ws' --track True
    parser = argparse.ArgumentParser()
    parser.add_argument("--working_directory", required=True)
    parser.add_argument("--track", default='False', type=lambda x: (str(x).lower() == 'true'))
    parser.add_argument("--gen_planos", default='False', type=lambda x: (str(x).lower() == 'true'))
    args = parser.parse_args()

    track_filter_and_count(args.working_directory, args.track, args.gen_planos)
