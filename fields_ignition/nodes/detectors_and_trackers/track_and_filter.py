#!/usr/bin/env python3

import os 
import subprocess
from detectors_and_trackers.filtrado.filtrado_kmeans import FiltradoKMeans
from detectors_and_trackers.filtrado.filtrado_plano import FiltradoPlano
from detectors_and_trackers.filtrado.sin_filtrado import SinFiltrado
from detectors_and_trackers.filtrado.filtrado_filas_posteriores import FiltradoFilasPosteriores
import argparse
import cv2
import numpy as np
from detectors_and_trackers.yolo_tracking.tracking.track import main as track_main

# importar configuraciones
import configparser

# directorio de trabajo
CWD = os.getcwd()

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
        self.config_path = config_path

        # Lectura de configuraciones
        self.image_height =             config.getint('TRACK_AND_FILTER', 'image_height')
        self.image_width =              config.getint('TRACK_AND_FILTER', 'image_width')
        self.tracking_method =          config.get('TRACK_AND_FILTER', 'tracking_method')
        self.yolo_weights =             config.get('TRACK_AND_FILTER', 'yolo_weights')
        self.world_name =               config.get('TRACK_AND_FILTER', 'world_name')
        self.track =                    config.getboolean('TRACK_AND_FILTER', 'track')
        self.gen_imagenes_tracker =     config.get('TRACK_AND_FILTER', 'gen_imagenes_tracker')
        self.generar_imagen_plano =     config.get('TRACK_AND_FILTER', 'generar_imagen_plano')
        self.rotar_imagenes =           config.getboolean('TRACK_AND_FILTER', 'troncos_horizontales')
        self.verbose =                  config.getboolean('TRACK_AND_FILTER', 'verbose')
        self.debug_plano =              config.getboolean('FILTRADO_PLANO', 'debug_plano')
        method =                        config.get('TRACK_AND_FILTER', 'method')

        self.yolo_instance = None
        self.ids_filtrados = set()

        if method not in METODOS_FILTRADO.keys():
            allowed_methods = ", ".join(METODOS_FILTRADO.keys())
            raise ValueError(f"Method {method} not recognized, please use one of the following: {allowed_methods}")
        
        self.filter_class = METODOS_FILTRADO[method]

    def __setup_env(self):
        # clone the repository
        subprocess.run(['pip', 'install', 'boxmot'])

    def __current_path(self):
        return os.path.dirname(os.path.realpath(__file__))

    # leer las bounding boxes del archivo de bounding boxes
    def read_bounding_boxes(self):
        dir_path = f"{self.__current_path()}/yolo_tracking/runs/track/exp/labels/"

        # Obtener los nombres de los archivos que describen las bounding boxes
        file_names = os.listdir(dir_path)

        bounding_boxes = {}

        for file_name in file_names:
            # Obtener el timestamp del nombre del archivo (ejemplo: 814572000000.png)
            timestamp = file_name.split(".")[0]

            # Obtener las bounding boxes del archivo
            with open(os.path.join(dir_path, file_name), 'r') as bb_file:
                lines = bb_file.readlines()

                for line in lines:
                    line_split = line.split(" ")

                    # Si la linea tiene 6 elementos, la bounding box tiene un id, de lo contrario es 0
                    # los ids 0 corresponden a tracks no confirmados
                    if len(line_split) < 6:
                        continue

                    bb_id = int(line_split[5][:-1])
                    x,y = line_split[1:3]

                    x = float(x)
                    y = float(y)

                    # Como los valores estan normalizados, necesitamos multiplicarlos por el tamaño de la imagen
                    x = x * self.image_width
                    y = y * self.image_height

                    x = int(x)
                    y = int(y)

                    # Crear una lista con el centro de la bounding box y el id de la bounding box que es lo que se guardara en el diccionario
                    bb_center = [x, y, bb_id]

                    if (timestamp in bounding_boxes):
                        bounding_boxes[timestamp].append(bb_center)
                    else:
                        bounding_boxes[timestamp] = [bb_center]

        # El valor de retorno es un diccionario con el timestamp como key y un array con los centros de las bounding boxes del frame correspondiente como valor, y como tercer valor, el id de la bounding box.
        return bounding_boxes

    # when the node ends track the apples and evaluate the tracking
    # def __total_amount_apples_for_trees_ids(self, ids):
    #     dir_path = f"{CWD}/src/apple_fields_ignition/fields_ignition/generated/{self.world_name}/apple_field/"
    #     apple_amount = 0
    #     dir_names_wanted = [f"apple_{x}" for x in ids]
    #     for root, dirs, files in os.walk(dir_path):
    #         for dir_name in dir_names_wanted:
    #             data_file = os.path.join(root, dir_name, 'markers.json')
    #             if os.path.exists(data_file):
    #                 with open(data_file, 'r') as f:
    #                     data = json.load(f)
    #                     apple_amount += data["count_apples"]
    #     return apple_amount

    def __delete_folder(self, folder_path):
        subprocess.run(['rm', '-rf', folder_path])

    def __configs_filtro(self):
        return {
            "working_directory": CWD,
            "generar_imagen_plano": self.generar_imagen_plano,
            "rotar_imagenes": self.rotar_imagenes,
            "verbose": self.verbose,
            "debug_plano": self.debug_plano,
            "config_path": self.config_path
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
                
    # cuando el nodo es detenido, corre el tracker y filtra los resultados
    def track_filter_and_count(self):
        self.__setup_env()

        os.chdir(CWD)
        if self.verbose:
          print('Trabajando dentro de directorio ', CWD)
        
        print('Running tracker and tracker evaluator...')
        

        if self.track:
            # borrar runs/track/exp por si habia una corrida anterior guardada
            self.__delete_folder(f"{self.__current_path()}/yolo_tracking/runs/track/exp")
            
            # correr el tracker
            track_main(args=self.__configs_yolo())

        # obtener las bounding boxes del archivo generado por el tracker
        bounding_boxes = self.read_bounding_boxes()

        # instanciar filtro
        filtro = self.filter_class(self.__configs_filtro())

        for timestamp in bounding_boxes:
            img_original = cv2.imread("left_rgb_images/" + str(timestamp) + ".png")
            mapa_profundidad = np.load("depth_matrix/" + str(timestamp) + ".npy")  

            filtered_bbs = filtro.filter(timestamp, bounding_boxes[timestamp], img_original, mapa_profundidad)

            for bb in filtered_bbs:
                self.ids_filtrados.add(bb[2])
        
        # Imprimir resultados
        print('Numero de manzanas detectado: ' + str(self.get_apple_count()))

        # print(f"amount of apples exactly for trees id (5,6,7,8,9): { self.__total_amount_apples_for_trees_ids([5,6,7,8,9])}")

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
