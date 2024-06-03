#!/usr/bin/env python3

import os 
import subprocess
from detectors_and_trackers.filtrado.filtrado_kmeans import FiltradoKMeans
from detectors_and_trackers.filtrado.filtrado_plano import FiltradoPlano
from detectors_and_trackers.filtrado.sin_filtrado import SinFiltrado
import argparse
import json
import cv2

ros_namespace = os.getenv('ROS_NAMESPACE')
image_height = 1024
image_width = 1024
offset_horizontal = 53
# global variables
TRACKING_METHOD = "deepocsort"
YOLO_WEIGHTS = "weights/yolov8l_150.pt"
WORLD_NAME = "stereo_trees_close"

class TrackAndFilter:
    def __init__(self, working_directory, method, track=False, gen_imagenes_tracker=False, generar_imagen_plano=False):
        self.working_directory = working_directory
        self.track = track
        self.gen_imagenes_tracker = gen_imagenes_tracker
        self.generar_imagen_plano = generar_imagen_plano

        if method == "kmeans":
            self.method = FiltradoKMeans
        elif method == "plano":
            self.method = FiltradoPlano
        elif method == "sin_filtrado":
            self.method = SinFiltrado
        else:
            raise ValueError(f"Method {method} not recognized")

    # clone repository with tracker and tracker evaluator - after running the node for the first time, run `pip install --upgrade sentry-sdk`
    def __clone_tracker_repo(self):
        subprocess.run(["git", "clone", "--recurse-submodules", "https://github.com/PaoloCappelli/yolo_tracking.git"])

        # in folder yolo_tracking run git pull to get the latest version
        os.chdir("yolo_tracking")
        
        subprocess.run(["git", "reset", "--hard","origin/master"])
        subprocess.run(["git", "pull"])

        # go back to the original directory and install the tracker
        os.chdir("..")
        subprocess.run(["pip", "install", "boxmot"])

    # read bounding boxes from the bounding box file
    def __read_bounding_boxes(self):

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

                    # Create a list with the bounding box center and the bounding box id which is what will be saved in the dictionary
                    bb_center = [x, y, bb_id] #EL + 30 PARA CONSIDERAR LA FRANJA NEGRA QUE SALE EN LAS IMAGENES DEPROFUNDIDAD

                    if (timestamp in bounding_boxes):
                        bounding_boxes[timestamp].append(bb_center)
                    else:
                        bounding_boxes[timestamp] = [bb_center]

        # The return value is a dictionary with the timestamp as the key and an array with the bounding boxes centers of the corresponding frame as the value, and as a third value, the bounding box id.
        return bounding_boxes

    # when the nodes ends track the apples and evaluate the tracking
    def __total_amount_apples_for_trees_ids(self, ids):
        dir_path = f"{self.working_directory}/src/apple_fields_ignition/fields_ignition/generated/{WORLD_NAME}/apple_field/"
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

    def __delete_folder(self, folder_path):
        subprocess.run(['rm', '-rf', folder_path])

    # when the node is killed, run the tracker and filter the results
    def track_filter_and_count(self):
        os.chdir(self.working_directory)
        print('working inside directory ', os.getcwd())
        
        self.__clone_tracker_repo()

        print('Running tracker and tracker evaluator...')
        SOURCE = "left_rgb_images"

        if self.track:
            self.__delete_folder('yolo_tracking/runs/track/exp')
            # run the tracker
            extra_args = []
            if self.gen_imagenes_tracker: extra_args = ["--save", "--show-conf"]

            subprocess.run(["python3", "yolo_tracking/tracking/track.py", "--yolo-model", YOLO_WEIGHTS, "--tracking-method", TRACKING_METHOD, "--source", SOURCE, "--save", "--save-txt", *extra_args]) 

        # get the bounding boxes from the file
        bounding_boxes = self.__read_bounding_boxes()

        # seteo configs para el filtrado
        configs = {
            "working_directory": self.working_directory,
            "generar_imagen_plano": self.generar_imagen_plano
        }

        # instancio filtro
        filtro = self.method(configs)
        
        bounding_boxes_filtrados = []

        for timestamp in bounding_boxes:
            
            img_original = cv2.imread("left_rgb_images/" + str(timestamp) + ".png")
            mapa_profundidad = cv2.imread("disparity_images/" + str(timestamp) + ".png", cv2.IMREAD_GRAYSCALE)   

            filtered_bbs = filtro.filter(timestamp, bounding_boxes[timestamp], img_original, mapa_profundidad)

            # print(f"Amount of apples in image: {len(image_depth_data)}, filtered apples: {len(filtered_depths)}")
            bounding_boxes_filtrados.extend(filtered_bbs)

        # Count the distinct ids that remained after filtering
        ids = []
        for bb in bounding_boxes_filtrados:
            ids.append(bb[2])
        ids = set(ids)
        
        # Print the number of apples
        print('Number of apples counted: ' + str(len(ids)))
        print(f"amount of apples exactly for trees id (5,6,7,8,9): { self.__total_amount_apples_for_trees_ids([5,6,7,8,9])}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--working_directory", required=True)
    parser.add_argument("--method", required=True)
    parser.add_argument("--track", default='False', type=lambda x: (str(x).lower() == 'true'))
    parser.add_argument("--gen_imagenes_tracker", default='False', type=lambda x: (str(x).lower() == 'true'))
    parser.add_argument("--generar_imagen_plano", default='False', type=lambda x: (str(x).lower() == 'true'))
    args = parser.parse_args()

    track_filter = TrackAndFilter(args.working_directory, args.method, args.track, args.gen_imagenes_tracker, args.generar_imagen_plano)
    track_filter.track_filter_and_count()

# export PYTHONPATH=/home/renzo/catkin_ws/src/apple_fields_ignition/fields_ignition/nodes
# python3 -m detectors_and_trackers.track_and_filter