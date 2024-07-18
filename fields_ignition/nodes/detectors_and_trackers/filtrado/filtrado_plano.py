#!/usr/bin/env python3

from . import filtrado_base
from ultralytics import YOLO
import cv2
import numpy as np
import os

# importar configuraciones
import configparser
config = configparser.ConfigParser()
config.read('src/apple_fields_ignition/fields_ignition/nodes/config.ini')

OFFSET_HORIZONTAL = config.getint('CONSTANTS', 'offset_horizontal')
MARGEN_PLANO = config.getint('CONSTANTS', 'margen_plano')
MODELO_PLANO = config.get('MODELO_PLANO', 'modelo_plano')

class CantidadPuntosInsuficiente(Exception):
    def __init__(self, m):
        self.message = m
    def __str__(self):
        return self.message
    

class FiltradoPlano(filtrado_base.FiltradoBase):
    def __init__(self, config):
        super().__init__(config)
        self.modelo_tronco = YOLO(f"{config['working_directory']}/weights/{MODELO_PLANO}.pt")
        self.__preparar_carpetas()

    def filter(self, timestamp, bounding_boxes, img_original, mapa_profundidad):
        filtered_points = []

        try:
            filtered_points = self.__filtrar_puntos(timestamp, bounding_boxes, img_original, mapa_profundidad)
        except CantidadPuntosInsuficiente as e:
            print(f"frame skipped, error: {e}")

        return filtered_points

    def __find_clusters(self, nums, threshold=10):
        if not nums:
            return 0

        # Sort the list of numbers
        sorted_nums = sorted(nums, reverse=True)
        maximo = sorted_nums[0]

        # Initialize the first group
        puntos_incluidos = [maximo]

        # Iterar a través de los números ordenados y formar grupos
        for i in range(1, len(sorted_nums)):
            if abs(maximo - sorted_nums[i]) > threshold:
                break

            puntos_incluidos.append(sorted_nums[i])
        
        return min(puntos_incluidos)

    def __obtener_puntos_arboles(self, img):
        puntos_arboles = {}

        results = self.modelo_tronco([img], iou=0.1, conf=0.35, show_conf=True, show_labels=False, show=False)

        # Visualizar resultados

        # Este metodo solo recibe una imagen, por lo que en el vector resultado del llamado
        # al modelo solo habra un resultado
        res = results[0]
        if res.masks == None or len(res.masks.xy) < 2:
            raise CantidadPuntosInsuficiente("Hay menos de 2 troncos en la imagen")
        
        for mask_id, mask in enumerate(res.masks.xy):
            # inicializo con una lista vacia
            puntos_arboles[mask_id] = []

            # obtener el centro de cada tronco
            x_center, y_center, _, _ = res.boxes[mask_id].xywh[0].cpu().numpy()
            
            cv2.circle(img, (int(x_center), int(y_center)), 3, (255, 255, 255), -1)

            # Inicializa una máscara binaria
            mask_binaria = np.zeros(img.shape[:2], dtype=np.uint8)

            # tuplas de los bordes de la mascara
            data_points = [tuple(elem) for elem in mask]
            data_points = np.array([data_points], dtype=np.int32)  # Convertir a matriz numpy de tipo int32

            # Pinta la máscara binaria
            cv2.fillPoly(mask_binaria, data_points, 1)

            # Encuentra las coordenadas de los píxeles dentro de la máscara
            y, x = np.where(mask_binaria == 1)
            pixeles_tronco_coords = list(zip(x, y))  # Convertir el zip en una lista

            # find the closest point in the mask array to the calculated center
            closest_point = pixeles_tronco_coords[0]
            for point in pixeles_tronco_coords:
                point_x, point_y = point[0], point[1]
                closest_x, closest_y = closest_point[0], closest_point[1]

                if abs(point_x - x_center) + abs(point_y - y_center) < abs(closest_x - x_center) + abs(closest_y - y_center):
                    closest_point = point
                
            # find the furthest point above and below the center
            above_point = closest_point
            below_point = closest_point
            closest_x, closest_y = closest_point[0], closest_point[1]

            for point in pixeles_tronco_coords:
                if point[1] <= closest_y and point[1] <= above_point[1]:
                    above_point = point
                if point[1] >= closest_y and point[1] >= below_point[1]:
                    below_point = point

            cv2.circle(img, (int(closest_point[0]), int(closest_point[1])), 3, (0, 0, 255), -1)
            cv2.circle(img, (int(above_point[0]), int(above_point[1])), 3, (0, 255, 0), -1)
            cv2.circle(img, (int(below_point[0]), int(below_point[1])), 3, (255, 0, 0), -1)

            # now find the point in the middle of center and above
            diff_x = (closest_x - above_point[0])/2
            diff_y = (closest_y - above_point[1])/2

            above_center_x = above_point[0] + diff_x
            above_center_y = above_point[1] + diff_y

            # now for below
            diff_x = (closest_x - below_point[0])/2
            diff_y = (closest_y - below_point[1])/2

            below_center_x = below_point[0] + diff_x
            below_center_y = below_point[1] + diff_y

            cv2.circle(img, (int(above_center_x), int(above_center_y)), 3, (0, 55, 0), -1)
            cv2.circle(img, (int(below_center_x), int(below_center_y)), 3, (55, 0, 0), -1)

            # search for points in pixeles_tronco_coords
            for point in pixeles_tronco_coords:
                point_x, point_y = point[0], point[1]
                above_x, above_y = above_point[0], above_point[1]
                below_x, below_y = below_point[0], below_point[1]

                if abs(point_x - below_center_x) + abs(point_y - below_center_y) <= abs(below_x - below_center_x) + abs(below_y - below_center_y):
                    below_point = point

                if abs(point_x - above_center_x) + abs(point_y - above_center_y) <= abs(above_x - above_center_x) + abs(above_y - above_center_y):
                    above_point = point


            cv2.circle(img, (int(above_point[0]), int(above_point[1])), 3, (0, 155, 0), -1)
            cv2.circle(img, (int(below_point[0]), int(below_point[1])), 3, (155, 0, 0), -1)

            puntos_arboles[mask_id].append(closest_point)
            puntos_arboles[mask_id].append(above_point)
            puntos_arboles[mask_id].append(below_point)               

        return puntos_arboles

    def __obtener_puntos_con_profunidad(self, puntos_arboles, mapa_profunidad):
        puntos_con_profundidad = {}
        for mask_id, puntos in puntos_arboles.items():
            for [x,y] in puntos:

                if x <= OFFSET_HORIZONTAL:
                    continue
                
                if mask_id not in puntos_con_profundidad: puntos_con_profundidad[mask_id] = [] 

                z = self.__escalar_profundidad(mapa_profunidad[y, x])
                puntos_con_profundidad[mask_id].append([x,y,z])

        return puntos_con_profundidad

    def __filtrar_puntos_threshold(self, puntos_arboles):
        puntos = []
        for mask_id, puntos_de_arbol in puntos_arboles.items():
            puntos += puntos_de_arbol

        threshold = int(self.__find_clusters([p[2] for p in puntos]))

        # print(f"threshold hallado: {threshold}")

        puntos_filtrados = {}
        for mask_id, puntos_de_arbol in puntos_arboles.items():
            for p in puntos_de_arbol:
                if p[2] >= threshold:
                    if mask_id not in puntos_filtrados: puntos_filtrados[mask_id] = []
                    puntos_filtrados[mask_id].append(p)

        return puntos_filtrados

    def __obtener_plano(self, puntos):
        if len(puntos) < 3:
            raise CantidadPuntosInsuficiente("Hay menos de 3 puntos, no se puede definir el plano")
        
        # print(f"Puntos del plano: {puntos}")
        A = np.array(puntos)
        b = np.ones(len(puntos))
        
        # Solve for the coefficients using least squares
        # The normal equation is (A.T * A) * x = A.T * b
        # We use np.linalg.lstsq to solve this equation which minimizes ||Ax - b||
        # Where x corresponds to the coefficients [a, b, c]
        coeffs, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        
        # The result is the coefficients a, b, c, where we have assumed d = 1
        a, b, c = coeffs
        d = -1  # We assume d = -1 to solve for a, b, c
        
        return a, b, c, d

    def __delante_de_plano(self, x, y, z, a, b, c, d):
        return a * x + b * y + c * z + d > 0

    def __escalar_profundidad(self, valor_z):
        # Invierte el valor z ya que en el mapa de profundidad, un valor más alto significa más cerca
        #return ((255 - valor_z) / 255) * (MAX_DEPTH - MIN_DEPTH) + MIN_DEPTH
        return valor_z

    def __visualizar_plano_en_imagen(self, img, puntos_manzanas, depth_map, a, b, c, d):
        # Crear una copia de la imagen para dibujar el plano
        img_with_plane = img.copy()

        for y in range(img.shape[0]):
            for x in range(img.shape[1]):
                # Asegúrate de que el índice no salga del rango de la imagen

                # Obtener la profundidad desde el mapa de profundidad
                z = depth_map[y, x]
                # Escala la profundidad al rango correcto
                z_scaled = self.__escalar_profundidad(z)

                # Comprobar si el punto está delante del plano
                esta_delante = self.__delante_de_plano(x, y, z_scaled, a, b, c, d)

                current_color = img_with_plane[y, x]
                img_with_plane[y, x] = current_color if esta_delante else (int(current_color[0]/2), int(current_color[1]/2), int(current_color[2]/2))
        for [x, y, *rest] in puntos_manzanas:
            # calcular profundidad de la manzana
            z = depth_map[y, x]
            z_scaled = self.__escalar_profundidad(z)

            # Comprobar si el punto está delante del plano
            esta_delante = self.__delante_de_plano(x, y, z_scaled, a, b, c, d)

            if esta_delante:
                cv2.circle(img_with_plane, (x, y), 3, (0, 255, 0), -1)
            else:
                cv2.circle(img_with_plane, (x, y), 3, (0, 0, 255), -1)

        return img_with_plane

    def __filtrar_puntos(self, timestamp, puntos_manzanas, img_original, mapa_profundidad):
        puntos_arboles = self.__obtener_puntos_arboles(img_original)
        puntos_con_profundidad = self.__obtener_puntos_con_profunidad(puntos_arboles, mapa_profundidad)
        puntos_filtrados = self.__filtrar_puntos_threshold(puntos_con_profundidad)

        numero_arboles = len(puntos_filtrados.keys())

        if numero_arboles < 2:
            raise CantidadPuntosInsuficiente("Luego de filtrado los puntos, no quedan 2 arboles")

        total_puntos = []
        for puntos_tronco in puntos_filtrados.values():
            total_puntos += puntos_tronco

        # vamos a correr el plano en z un margen
        total_puntos = [[x, y, z + MARGEN_PLANO] for x, y, z in total_puntos]

        a, b, c, d = self.__obtener_plano(total_puntos)

        print('plano generado: ')
        print(f"a: {a}, b: {b}, c: {c}, d: {d}")

        if self.config["generar_imagen_plano"]:
            img_with_plane = self.__visualizar_plano_en_imagen(img_original, puntos_manzanas, mapa_profundidad, a, b, c, d)
            cv2.imwrite(f"{self.config['working_directory']}/planos/pixeles_filtrados_{timestamp}.png", img_with_plane)

        puntos_filtrados = []

        for [x, y, *rest] in puntos_manzanas:
            if x <= OFFSET_HORIZONTAL:
                continue

            # Obtener la profundidad desde el mapa de profundidad
            z = mapa_profundidad[y, x]
            # Escala la profundidad al rango correcto
            z_scaled = self.__escalar_profundidad(z)

            # Comprobar si el punto está delante del plano
            esta_delante = self.__delante_de_plano(x, y, z_scaled, a, b, c, d)

            if esta_delante:
                puntos_filtrados.append([x, y, *rest])

        return puntos_filtrados

    def __preparar_carpetas(self):
        working_directory = self.config['working_directory']

        # si la carpeta no existe, la crea, sino se vacia
        if not os.path.exists(f"{working_directory}/planos"):
            os.makedirs(f"{working_directory}/planos")
        else:
            for file in os.listdir(f"{working_directory}/planos"):
                os.remove(f"{working_directory}/planos/{file}")

        return working_directory