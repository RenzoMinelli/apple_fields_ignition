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

cwd = os.getcwd()

image_height = 1024
image_width = 1024

METODOS_FILTRADO = {
    "kmeans": FiltradoKMeans, 
    "plano": FiltradoPlano, 
    "sin_filtrado": SinFiltrado, 
    "filas_posteriores": FiltradoFilasPosteriores 
}

class Plotting:
  # read bounding boxes from the bounding box file
    def read_bounding_boxes(self):

        # Specify dir path
        dir_path = f"{cwd}/src/apple_fields_ignition/fields_ignition/nodes/detectors_and_trackers/yolo_tracking/runs/track/exp/labels/"

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
    def draw_boxes_and_save(self, image_path, green_bboxs, red_bboxs, output_folder):
        # Read the image
        img = cv2.imread(image_path)

        # Iterate over each RED bounding box in the list
        for i, bbox in enumerate(red_bboxs):

            x, y, _, _ = bbox

            # Draw the bounding box
            cv2.circle(img, (int(x), int(y)), 2, (0, 0, 255), 2)

        # Iterate over each GREEN bounding box in the list
        for i, bbox in enumerate(green_bboxs):

            x, y, _, _ = bbox

            # Draw the bounding box
            cv2.circle(img, (int(x), int(y)), 2, (0, 255, 0), 2)

        # Save the modified image with bounding boxes
        filename = os.path.basename(image_path)
        output_path = os.path.join(output_folder, filename)

        # debug log
        print('image: ', image_path)
        print('saving image with bounding boxes in ', output_path)

        cv2.imwrite(output_path, img)


    # when the node is killed, run the tracker and filter the results
    def track_filter_and_count(self, working_directory, filter_class):
        os.chdir(working_directory)
        print('working inside directory ', os.getcwd())
        
        # get the bounding boxes from the file
        # dictionary with the timestamp as the key and an array with the bounding boxes centers of the corresponding frame as the value, and as a third value, the bounding box id.
        # bounding_boxes = {<timestamp>: [[<x>, <y>, <id>, <w>, <h>], ...], ...}
        bounding_boxes = self.read_bounding_boxes()

        for timestamp in bounding_boxes:

            mapa_profundidad = cv2.imread("disparity_images/" + str(timestamp) + ".png", cv2.IMREAD_GRAYSCALE)
            
            # instancio el filtro
            filtro = filter_class()

            # Estas las imprimimos todas en rojo
            red_depths = filtro.obtener_puntos_con_profunidad(bounding_boxes[timestamp], mapa_profundidad)
            # Luego sobreescribimos las que no seran descartadas con color verde.
            green_depths = filtro.filter(None, bounding_boxes[timestamp], None, mapa_profundidad)

            image_path = 'left_rgb_images/' + timestamp + '.png'
            output_folder = working_directory + '/test_filtered_images'
            self.draw_boxes_and_save(image_path, green_depths, red_depths, output_folder)


def plot():
    parser = argparse.ArgumentParser()
    parser.add_argument("--working_directory", required=True)
    parser.add_argument("--method", required=True)
    args = parser.parse_args()

    folder_names = ["test_depth_i10", "test_filtered_images"]
    for folder_name in folder_names:
        folder_path = f"{args.working_directory}/{folder_name}"
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        else:
            files = os.listdir(folder_path)
            for file_name in files:
                file_path = os.path.join(folder_path, file_name)
                os.remove(file_path)

    plotting = Plotting()

    filter_class = None
    method = args.method
    if method not in METODOS_FILTRADO.keys():
        allowed_methods = ", ".join(METODOS_FILTRADO.keys())
        raise ValueError(f"Method {method} not recognized, please use one of the following: {allowed_methods}")
    
    filter_class = METODOS_FILTRADO[method]
    plotting.track_filter_and_count(args.working_directory, filter_class)

if __name__ == "__main__":
    plot()