#!/usr/bin/env python3

# imports
import cv2
import os
import numpy
import argparse
from detectors_and_trackers.filtrado.filtrado_kmeans import FiltradoKMeans
from detectors_and_trackers.filtrado.filtrado_plano import FiltradoPlano
from detectors_and_trackers.filtrado.sin_filtrado import SinFiltrado
from detectors_and_trackers.filtrado.filtrado_filas_posteriores import FiltradoFilasPosteriores
import configparser
config = configparser.ConfigParser()

CWD = os.getcwd()

image_height = 1024
image_width = 1024

METODOS_FILTRADO = {
    "kmeans": FiltradoKMeans, 
    "plano": FiltradoPlano, 
    "sin_filtrado": SinFiltrado, 
    "filas_posteriores": FiltradoFilasPosteriores 
}

class Plotting:
  # Leer los bounding boxes del archivo de bounding boxes
    def read_bounding_boxes(self):

        # Specify dir path
        dir_path = f"{CWD}/src/apple_fields_ignition/fields_ignition/nodes/detectors_and_trackers/yolo_tracking/runs/track/exp/labels/"

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

    # Define a function to draw bounding boxes on an image and save the modified image
    def draw_boxes_and_save(self, timestamp, imagen_original, green_bboxs, red_bboxs, output_folder):
        # Iterate over each RED bounding box in the list
        for i, [x,y,*rest] in enumerate(red_bboxs):

            # Draw the bounding box
            cv2.circle(imagen_original, (int(x), int(y)), 2, (0, 0, 255), 2)

        # Iterate over each GREEN bounding box in the list
        for i, [x,y,*rest] in enumerate(green_bboxs):

            # Draw the bounding box
            cv2.circle(imagen_original, (int(x), int(y)), 2, (0, 255, 0), 2)

        # Save the modified image with bounding boxes
        output_path = os.path.join(output_folder, f"{timestamp}.png")

        # debug log
        print('saving image with bounding boxes in ', output_path)

        cv2.imwrite(output_path, imagen_original)


    # when the node is killed, run the tracker and filter the results
    def track_filter_and_count(self, working_directory, filter_class, config_path, rotar_imagenes):
        os.chdir(working_directory)
        print('working inside directory ', os.getcwd())
        
        # get the bounding boxes from the file
        # dictionary with the timestamp as the key and an array with the bounding boxes centers of the corresponding frame as the value, and as a third value, the bounding box id.
        # bounding_boxes = {<timestamp>: [[<x>, <y>, <id>, <w>, <h>], ...], ...}
        bounding_boxes = self.read_bounding_boxes()

        for timestamp in bounding_boxes:

            mapa_profundidad = cv2.imread("disparity_images/" + str(timestamp) + ".png", cv2.IMREAD_GRAYSCALE)
            imagen_original = cv2.imread("left_rgb_images/" + str(timestamp) + ".png")

            configs_filtros = {
                "verbose": True,
                "debug_plano": False,
                "working_directory": CWD,
                "generar_imagen_plano": False,
                "rotar_imagenes": rotar_imagenes,
                "config_path": config_path
            }
            # instancio el filtro
            filtro = filter_class(configs_filtros)

            # Estas las imprimimos todas en rojo
            red_depths = filtro.obtener_puntos_con_profunidad(bounding_boxes[timestamp], mapa_profundidad)
            # Luego sobreescribimos las que no seran descartadas con color verde.
            green_depths = filtro.filter(timestamp, bounding_boxes[timestamp], imagen_original, mapa_profundidad)

            output_folder = working_directory + '/test_filtered_images'
            self.draw_boxes_and_save(timestamp, imagen_original, green_depths, red_depths, output_folder)


def plot():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    config.read(args.config)
    method = config.get('TRACK_AND_FILTER', 'method')
    rotar_imagenes = config.getboolean('TRACK_AND_FILTER', 'troncos_horizontales')

    folder_path = f"{CWD}/test_filtered_images"
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    else:
        files = os.listdir(folder_path)
        for file_name in files:
            file_path = os.path.join(folder_path, file_name)
            os.remove(file_path)

    plotting = Plotting()

    filter_class = None
    if method not in METODOS_FILTRADO.keys():
        allowed_methods = ", ".join(METODOS_FILTRADO.keys())
        raise ValueError(f"Metodo de filtrado {method} no reconocido, usar alguno de los siguientes:\n {allowed_methods}")
    
    filter_class = METODOS_FILTRADO[method]
    plotting.track_filter_and_count(CWD, filter_class, args.config, rotar_imagenes)

if __name__ == "__main__":
    plot()

# python -m detectors_and_trackers.plot_filtered_apples --working_directory /home/renzo/catkin_ws --method plano --config /home/renzo/catkin_ws/src/apple_fields_ignition/fields_ignition/nodes/config.ini
