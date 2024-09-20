#!/usr/bin/env python3

import os 
import subprocess
from detectors_and_trackers.filtrado.filtrado_kmeans import FiltradoKMeans
from detectors_and_trackers.filtrado.filtrado_plano import FiltradoPlano
from detectors_and_trackers.filtrado.sin_filtrado import SinFiltrado
from detectors_and_trackers.filtrado.filtrado_punto_medio import FiltradoPuntoMedio
from detectors_and_trackers.filtrado.filtrado_filas_posteriores import FiltradoFilasPosteriores
import argparse
import cv2
import numpy as np
from detectors_and_trackers.yolo_tracking.tracking.track import main as track_main
import time 
import joblib

# importar configuraciones
import configparser

# directorio de trabajo
CWD = os.getcwd()

SOURCE = "rgb_images"

METODOS_FILTRADO = {
    "kmeans": FiltradoKMeans, 
    "plano": FiltradoPlano, 
    "sin_filtrado": SinFiltrado, 
    "filas_posteriores": FiltradoFilasPosteriores,
    "punto_medio": FiltradoPuntoMedio
}

class TrackAndFilter:
    def __init__(self, config_path, bag_name="unknown"):
        
        config = configparser.ConfigParser()
        config.read(config_path)
        self.config_path = config_path

        # Lectura de configuraciones
        self.image_height =             config.getint('TRACK_AND_FILTER', 'image_height')
        self.image_width =              config.getint('TRACK_AND_FILTER', 'image_width')
        self.count_threshold =          config.getint('TRACK_AND_FILTER', 'count_threshold')
        self.tracking_method =          config.get('TRACK_AND_FILTER', 'tracking_method')
        self.yolo_weights =             config.get('TRACK_AND_FILTER', 'yolo_weights')
        self.track =                    config.getboolean('TRACK_AND_FILTER', 'track')
        self.gen_imagenes_tracker =     config.getboolean('TRACK_AND_FILTER', 'gen_imagenes_tracker')
        self.generar_imagen_plano =     config.getboolean('FILTRADO_PLANO', 'generar_imagen_plano')
        self.rotar_imagenes =           config.getboolean('TRACK_AND_FILTER', 'troncos_horizontales')
        self.verbose =                  config.getboolean('TRACK_AND_FILTER', 'verbose')
        self.debug_plano =              config.getboolean('FILTRADO_PLANO', 'debug_plano')
        self.method =                   config.get('TRACK_AND_FILTER', 'method')
        self.coeficiente_de_ajuste =    config.get('TRACK_AND_FILTER', 'coeficiente_de_ajuste')
        self.modelo_de_regresion = config.get('TRACK_AND_FILTER', 'modelo_de_regresion')
        self.bag_name =                 bag_name

        self.count = None

        self.yolo_instance = None
        self.ids_filtrados = {}

        if self.method not in METODOS_FILTRADO.keys():
            allowed_methods = ", ".join(METODOS_FILTRADO.keys())
            raise ValueError(f"Method {self.method} not recognized, please use one of the following: {allowed_methods}")
        
        self.filter_class = METODOS_FILTRADO[self.method]

        # guardamos el tipo de camara para guardar los resultados con un nombre mas descriptivo
        offset = config.getint('CONSTANTS', 'offset_horizontal')
        self.camera_type = "depth" if offset == -1 else "stereo"

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
            "--save-txt",
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
    
    def __aumentar_conteo_de_id(self, id):
        if id in self.ids_filtrados:
            self.ids_filtrados[id] += 1
        else:
            self.ids_filtrados[id] = 1

    def __cargar_modelo_regresion(self, model_path):
        # Verificar si el archivo existe
        if os.path.exists(model_path):
            # Cargar el modelo usando joblib
            modelo = joblib.load(model_path)
            return modelo
        else:
            print(f"El archivo {model_path} no existe.")
            return None

    def __predecir_con_regresion(self, numero_de_manzanas_detectado):
        modelos_de_regresion = '/src/apple_fields_ignition/fields_ignition/scripts/ejecuciones_coeficiente_y_RL/modelos_regresion/'
        modelo_seleccionado = CWD + modelos_de_regresion + self.modelo_de_regresion + '.pkl'
        modelo_regresion = self.__cargar_modelo_regresion(modelo_seleccionado)
        
        valor_x = np.array(numero_de_manzanas_detectado).reshape(-1, 1)
        return round(modelo_regresion.predict(valor_x)[0])

    def __prediccion_con_coeficiente(self, numero_de_manzanas_detectado):
        return round(numero_de_manzanas_detectado * float(self.coeficiente_de_ajuste))

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

        total = len(bounding_boxes)
        count = 0
        time_start = time.time()

        for timestamp in bounding_boxes:
            count += 1
            time_elapsed = time.time() - time_start
            amount_missing = total - count
            average_time = time_elapsed / count
            time_remaining = (average_time * amount_missing)
            print(f"timestamp: {timestamp}, {count}/{total}, time remaining: {time_remaining} seconds")

            img_original = cv2.imread("rgb_images/" + str(timestamp) + ".png")
            mapa_profundidad = np.load("depth_maps/" + str(timestamp) + ".npy")  

            filtered_bbs = filtro.filter(timestamp, bounding_boxes[timestamp], img_original, mapa_profundidad)

            for bb in filtered_bbs:
                self.__aumentar_conteo_de_id(bb[2])
        
        # Imprimir resultados
        print('Numero de manzanas detectado: ' + str(self.apple_count()))
        
        # Predecir el valor real utilizando el ajuste por regresion
        prediccion_de_regresion = self.__predecir_con_regresion(self.apple_count())
        print('Numero de manzanas predicho por la regresion: ' + str(prediccion_de_regresion))

        # Predecir el valor real utilizando el ajuste por coeficiente
        prediccion_de_coeficiente = self.__prediccion_con_coeficiente(self.apple_count())
        print('Numero de manzanas ajustando por coeficiente: ' + str(prediccion_de_coeficiente))

        self.__save_results()

    def track_filter_and_count_one_frame(self, timestamp, img_original, mapa_profundidad):
        new_yolo_instance, bounding_boxes = track_main(args=self.__configs_yolo_one_frame(), image=img_original, yolo_model_instance=self.yolo_instance)
        self.yolo_instance = new_yolo_instance

        # instancio filtro
        filtro = self.filter_class(self.__configs_filtro())

        filtered_bbs = filtro.filter(timestamp, bounding_boxes, img_original, mapa_profundidad)

        for bb in filtered_bbs:
            self.__aumentar_conteo_de_id(bb[2])

    def apple_count(self):
        # count_threshold es el umbral de veces que un id debe aparecer para ser contado
        if self.count != None:
          return self.count
        
        count = 0
        for id in self.ids_filtrados:
            if self.ids_filtrados[id] >= self.count_threshold:
                count += 1
        self.count = count
        return count
    
    def __save_results(self):
        # creo la carpeta results si no existe
        if not os.path.exists(f"{CWD}/results"):
            os.makedirs(f"{CWD}/results")

        # genero un archivo con el content del config usado + el conteo en una nueva linea
        conteo = self.apple_count()
        results_path = f"{CWD}/results/{self.camera_type}_{self.method}_{self.bag_name}_{int(time.time())}.ini"

        print('Guardando resultados de ejecucion en: \n' + results_path)

        with open(self.config_path, "r") as config_file:
            content = config_file.read()
            with open(results_path, "w") as results_file:
                results_file.write(content)
                # Escribo el resultado normal
                results_file.write(f"[RESULTS]\napple_count = {conteo}\n")

                # Escribo el resultado ajustando con modelo de regresion
                ajuste_con_regresion = self.__predecir_con_regresion(self.apple_count())
                results_file.write(f"[RESULTS_WITH_REGRESSION]\napple_count = {str(ajuste_con_regresion)}\n")

                # Escribo el resultado ajustando por coeficiente
                ajuste_con_coeficiente = self.__prediccion_con_coeficiente(self.apple_count())
                results_file.write(f"[RESULTS_WITH_COEFFICIENT]\napple_count = {str(ajuste_con_coeficiente)}\n")
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--bag_name", required=False, default="unknown")
    args = parser.parse_args()

    track_filter = TrackAndFilter(args.config, args.bag_name)
    track_filter.track_filter_and_count()

# 'poetry shell' dentro de /home/<user>/catkin_ws/yolo_tracking
# 
# export PYTHONPATH=/home/<user>/catkin_ws/src/apple_fields_ignition/fields_ignition/nodes
# python3 -m detectors_and_trackers.track_and_filter --config <path to config.ini>
# EJEMPLO
# python3 -m detectors_and_trackers.track_and_filter --config /home/pincho/catkin_ws/src/apple_fields_ignition/fields_ignition/nodes/config.ini
