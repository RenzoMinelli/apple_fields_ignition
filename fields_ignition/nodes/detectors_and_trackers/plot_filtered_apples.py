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
from detectors_and_trackers.track_and_filter import TrackAndFilter
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
    # Define a function to draw bounding boxes on an image and save the modified image
    def draw_boxes_and_save(self, timestamp, imagen_original, green_bboxs, red_bboxs, output_folder):
        # Iterar sobre cada bounding box ROJA en la lista
        for i, [x,y,*rest] in enumerate(red_bboxs):
            # Dibujar la bounding box
            cv2.circle(imagen_original, (int(x), int(y)), 2, (0, 0, 255), 2)

        # Iterar sobre cada bounding box VERDE en la lista
        for i, [x,y,*rest] in enumerate(green_bboxs):
            # Dibujar la bounding box
            cv2.circle(imagen_original, (int(x), int(y)), 2, (0, 255, 0), 2)

        # Guardar la imagen modificada con las bounding boxes
        output_path = os.path.join(output_folder, f"{timestamp}.png")
        print('Saving image with bounding boxes in ', output_path)
        cv2.imwrite(output_path, imagen_original)

    def filter_and_plot(self, working_directory, filter_class, config_path, rotar_imagenes):
        os.chdir(working_directory)
        print('working inside directory ', os.getcwd())
        
        # para cada timestamp obtener las bounding boxes en un formato:
        # bounding_boxes = {<timestamp1>: [[<x>, <y>, <id>], ...], ...}
        bounding_boxes = TrackAndFilter(config_path).read_bounding_boxes()

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
    plotting.filter_and_plot(CWD, filter_class, args.config, rotar_imagenes)

if __name__ == "__main__":
    plot()

# Ejemplo de ejecucion
# python -m detectors_and_trackers.plot_filtered_apples --config <path al archivo de configuracion>
