#!/usr/bin/env python3

import os 
import subprocess
from detectors_and_trackers.filtrado.filtrado_kmeans import FiltradoKMeans
from detectors_and_trackers.filtrado.filtrado_plano import FiltradoPlano
from detectors_and_trackers.filtrado.sin_filtrado import SinFiltrado
from detectors_and_trackers.filtrado.filtrado_filas_posteriores import FiltradoFilasPosteriores
import argparse
import json
import cv2
from detectors_and_trackers.yolo_tracking.tracking.track import main as track_main

# importar configuraciones
import configparser

# directorio de trabajo
CWD = os.getcwd()

ros_namespace = os.getenv('ROS_NAMESPACE')

SOURCE = "left_rgb_images"

METODOS_FILTRADO = {
    "kmeans": FiltradoKMeans, 
    "plano": FiltradoPlano, 
    "sin_filtrado": SinFiltrado, 
    "filas_posteriores": FiltradoFilasPosteriores 
}

class TrackAndFilter:
    def __init__(self, config_path):
        
        config = configparser.ConfigParser()
        config.read(config_path)

        self.image_height = config.getint('TRACK_AND_FILTER', 'image_height')
        self.image_width = config.getint('TRACK_AND_FILTER', 'image_width')

        self.tracking_method = config.get('TRACK_AND_FILTER', 'tracking_method')
        self.yolo_weights = config.get('TRACK_AND_FILTER', 'yolo_weights')
        self.world_name = config.get('TRACK_AND_FILTER', 'world_name')

        self.track = config.getboolean('TRACK_AND_FILTER', 'track')
        self.gen_imagenes_tracker = config.get('TRACK_AND_FILTER', 'gen_imagenes_tracker')
        self.generar_imagen_plano = config.get('TRACK_AND_FILTER', 'generar_imagen_plano')
        self.rotar_imagenes = config.getboolean('TRACK_AND_FILTER', 'troncos_horizontales')
        self.verbose = config.getboolean('TRACK_AND_FILTER', 'verbose')
        self.debug_plano = config.getboolean('FILTRADO_PLANO', 'debug_plano')

        self.yolo_instance = None
        self.ids_filtrados = set()

        method = config.get('TRACK_AND_FILTER', 'method')
        if method not in METODOS_FILTRADO.keys():
            allowed_methods = ", ".join(METODOS_FILTRADO.keys())
            raise ValueError(f"Method {method} not recognized, please use one of the following: {allowed_methods}")
        
        self.filter_class = METODOS_FILTRADO[method]

    def __setup_env(self):
        # clone the repository
        subprocess.run(['pip', 'install', 'boxmot'])

    def __current_path(self):
        return os.path.dirname(os.path.realpath(__file__))

    # read bounding boxes from the bounding box file
    def __read_bounding_boxes(self):

        # Get all detections except the file right_rgb_images.txt

        # Specify dir path
        dir_path = f"{self.__current_path()}/yolo_tracking/runs/track/exp/labels/"

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
                    x = x * self.image_width # ESTO HARDCODEADO NO ME PARECE MUCHO PORQUE SI ALGUIEN EN EL FUTURO QUIERE CAMBIAR EL SENSOR SE COMPLICA REVISAR EL CODIGO, ME PARECE QUE DEBERIA SER UN PARAMETRO O UNA VARIABLE GLOABL MINIMO
                    y = y * self.image_height

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
        dir_path = f"{CWD}/src/apple_fields_ignition/fields_ignition/generated/{self.world_name}/apple_field/"
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

    def __configs_filtro(self):
        return {
            "working_directory": CWD,
            "generar_imagen_plano": self.generar_imagen_plano,
            "rotar_imagenes": self.rotar_imagenes,
            "verbose": self.verbose,
            "debug_plano": self.debug_plano
        }
    
    def __configs_yolo(self):
        extra_args = []
        if self.gen_imagenes_tracker: extra_args = ["--save", "--show-conf"]
        
        model_args = [
            "--yolo-model", f"{CWD}/weights/{self.yolo_weights}",
            "--tracking-method", self.tracking_method,
            "--source", SOURCE,
            "--save", "--save-txt",
            *extra_args
        ]

        if not self.verbose:
            model_args.extend(["--verbose"])

        return model_args

    def __configs_yolo_one_frame(self):
        model_args = [
            "--yolo-model", f"{CWD}/weights/{self.yolo_weights}",
            "--tracking-method", self.tracking_method,
            "--exist-ok"
        ]

        if not self.verbose:
            model_args.extend(["--verbose"])

        return model_args
                
    # when the node is killed, run the tracker and filter the results
    def track_filter_and_count(self):
        self.__setup_env()

        os.chdir(CWD)
        print('working inside directory ', os.getcwd())
        
        print('Running tracker and tracker evaluator...')
        

        if self.track:
            self.__delete_folder(f"{self.__current_path()}/yolo_tracking/runs/track/exp")
            # run the tracker
           
            track_main(args=self.__configs_yolo())

        # get the bounding boxes from the file
        bounding_boxes = self.__read_bounding_boxes()

        # instancio filtro
        filtro = self.filter_class(self.__configs_filtro())

        for timestamp in bounding_boxes:
            
            img_original = cv2.imread("left_rgb_images/" + str(timestamp) + ".png")
            mapa_profundidad = cv2.imread("disparity_images/" + str(timestamp) + ".png", cv2.IMREAD_GRAYSCALE)   

            filtered_bbs = filtro.filter(timestamp, bounding_boxes[timestamp], img_original, mapa_profundidad)

            # print(f"Amount of apples in image: {len(image_depth_data)}, filtered apples: {len(filtered_depths)}")
            for bb in filtered_bbs:
                self.ids_filtrados.add(bb[2])
        
        # Print the number of apples
        print('Number of apples counted: ' + str(self.get_apple_count()))
        print(f"amount of apples exactly for trees id (5,6,7,8,9): { self.__total_amount_apples_for_trees_ids([5,6,7,8,9])}")

    def track_filter_and_count_one_frame(self, timestamp, img_original, mapa_profundidad):
        new_yolo_instance, bounding_boxes = track_main(args=self.__configs_yolo_one_frame(), image=img_original, yolo_model_instance=self.yolo_instance)
        self.yolo_instance = new_yolo_instance

        # instancio filtro
        filtro = self.filter_class(self.__configs_filtro())

        filtered_bbs = filtro.filter(timestamp, bounding_boxes, img_original, mapa_profundidad)

        for bb in filtered_bbs:
            self.ids_filtrados.add(bb[2])

    def get_apple_count(self):
        return len(self.ids_filtrados)
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    track_filter = TrackAndFilter(args.config)
    track_filter.track_filter_and_count()

# 'poetry shell' dentro de /home/<user>/catkin_ws/yolo_tracking
# 
# export PYTHONPATH=/home/<user>/catkin_ws/src/apple_fields_ignition/fields_ignition/nodes
# python3 -m detectors_and_trackers.track_and_filter --config <path to config.ini>
# EJEMPLO
# python3 -m detectors_and_trackers.track_and_filter --config /home/pincho/catkin_ws/src/apple_fields_ignition/fields_ignition/nodes/config.ini
