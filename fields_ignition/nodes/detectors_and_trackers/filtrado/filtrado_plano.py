#!/usr/bin/env python3

from .filtrado_base import FiltradoBase
from .filtrado_filas_posteriores import FiltradoFilasPosteriores
from ultralytics import YOLO
import cv2
import numpy as np
import os

CWD = os.getcwd()

class CantidadPuntosInsuficiente(Exception):
    def __init__(self, m):
        self.message = m
    def __str__(self):
        return self.message

class FiltradoPlano(FiltradoBase):
    def __init__(self, config={}):
        super().__init__(config)
        self.modelo_tronco = YOLO(f"{CWD}/weights/{self.imported_config.get('FILTRADO_PLANO', 'modelo_tronco')}.pt")
        self.FiltradoPlano_OFFSET_HORIZONTAL = self.imported_config.getint('CONSTANTS', 'offset_horizontal')
        self.FiltradoPlano_MARGEN_PLANO = self.imported_config.getfloat('FILTRADO_PLANO', 'margen_plano')
        self.FiltradoPlano_GENERAR_IMAGEN_PLANO = self.imported_config.getboolean('FILTRADO_PLANO', 'generar_imagen_plano')
        self.filtro_filas_posteriores = FiltradoFilasPosteriores(config)
        self.__preparar_carpetas()

    def filter(self, timestamp, bounding_boxes, img_original, mapa_profundidad):
        filtered_points = []

        try:
            filtered_points = self.__filtrar_puntos(timestamp, bounding_boxes, img_original, mapa_profundidad)
        except CantidadPuntosInsuficiente as e:
            if self.config["verbose"]:
                print(f"frame skippeado, error: {e}")

        return filtered_points

    # obtiene 3 puntos reales dentro del tronco, uno central, uno arriba y uno abajo
    # ademas permite la visualizacion de esos puntos mediante cv2
    def __obtener_puntos_arboles(self, img_orig, timestamp):
        puntos_arboles = {}

        if self.config["rotar_imagenes"]:
            img = cv2.rotate(img_orig, cv2.ROTATE_90_CLOCKWISE)
        else:
            img = img_orig

        results = self.modelo_tronco([img], iou=0.1, conf=0.35,show_conf=True,show_labels=False,show=False,verbose=self.config["verbose"])

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
            points_in_image = []
            data_points = [tuple(elem) for elem in mask]

            # remover puntos que estan fuera de la imagen (bug de yolo)
            for pair in data_points:
                if pair[0] < img.shape[:2][1] and pair[1] < img.shape[:2][0]:
                    points_in_image.append(pair)

            if len(points_in_image) == 0:
                continue

            data_points = np.array([points_in_image], dtype=np.int32) # Convertir a matriz numpy de tipo int32

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

            
            

            # ahora encuentra el punto en el medio del centro y arriba
            # esto es porque si usamos el punto más lejano arriba (o más lejano abajo),
            # más tarde, la imagen de disparidad
            # puede dar una medición incorrecta porque el punto está demasiado
            # cerca del borde del tronco
            diff_x = (closest_x - above_point[0])/2
            diff_y = (closest_y - above_point[1])/2

            above_center_x = above_point[0] + diff_x
            above_center_y = above_point[1] + diff_y

            # now for below
            diff_x = (closest_x - below_point[0])/2
            diff_y = (closest_y - below_point[1])/2

            below_center_x = below_point[0] + diff_x
            below_center_y = below_point[1] + diff_y

            # buscar los puntos reales del tronco que estan mas cerca
            # de los puntos calculados
            for point in pixeles_tronco_coords:
                point_x, point_y = point[0], point[1]
                above_x, above_y = above_point[0], above_point[1]
                below_x, below_y = below_point[0], below_point[1]

                if abs(point_x - below_center_x) + abs(point_y - below_center_y) <= abs(below_x - below_center_x) + abs(below_y - below_center_y):
                    below_point = point

                if abs(point_x - above_center_x) + abs(point_y - above_center_y) <= abs(above_x - above_center_x) + abs(above_y - above_center_y):
                    above_point = point

            # ahora que tenemos los puntos arriba centro abajo, vamos a agarrar el punto mas en el medio del tronco en cada fila
            puntos_centrados_horizontal = []
            for punto in [closest_point, above_point, below_point]:
                x, y = punto
                # obtener todos los puntos en la misma fila
                puntos_misma_fila = [p for p in pixeles_tronco_coords if p[1] == y]

                # encontrar el punto mas a la izquierda que es 1
                # luego el punto mas a la derecha que es 1
                # y sacar el punto en el medio
                ind_inicio = min([p[0] for p in puntos_misma_fila])
                ind_fin = max([p[0] for p in puntos_misma_fila])
                diff = ind_fin - ind_inicio
                ind_centro = int(ind_inicio + diff/2)

                # ahora queremos el punto mas cercano a (ind_centro, y)
                punto_en_mascara = None
                for point in puntos_misma_fila:
                    if not punto_en_mascara:
                        punto_en_mascara = point
                        continue

                    if abs(point[0] - ind_centro) < abs(punto_en_mascara[0] - ind_centro):
                        punto_en_mascara = point

                puntos_centrados_horizontal.append(punto_en_mascara)

            closest_point = puntos_centrados_horizontal[0]
            above_point = puntos_centrados_horizontal[1]
            below_point = puntos_centrados_horizontal[2]

            cv2.circle(img, (int(closest_point[0]), int(closest_point[1])), 3, (255, 0, 0), -1)
            cv2.circle(img, (int(above_point[0]), int(above_point[1])), 3, (242, 5, 211), -1)
            cv2.circle(img, (int(below_point[0]), int(below_point[1])), 3, (3, 255, 251), -1)

            if self.config["rotar_imagenes"]:
                # rotar anti horario
                closest_point = (closest_point[1], img.shape[1] - closest_point[0])
                above_point = (above_point[1], img.shape[1] - above_point[0])
                below_point = (below_point[1], img.shape[1] - below_point[0])

                # pintar circulos en imagen original
                cv2.circle(img_orig, (int(closest_point[0]), int(closest_point[1])), 3, (255, 0, 0), -1)
                cv2.circle(img_orig, (int(above_point[0]), int(above_point[1])), 3, (242, 5, 211), -1)
                cv2.circle(img_orig, (int(below_point[0]), int(below_point[1])), 3, (3, 255, 251), -1)

            puntos_arboles[mask_id].append(closest_point)
            puntos_arboles[mask_id].append(above_point)
            puntos_arboles[mask_id].append(below_point)               

            if self.config["debug_plano"]:
                cv2.imwrite(f"{CWD}/puntos_para_plano/{timestamp}.png", img)

        return puntos_arboles

    def __obtener_puntos_con_profunidad(self, puntos_arboles, mapa_profunidad):
        puntos_con_profundidad = []
        for [x,y] in puntos_arboles:

            if x <= self.FiltradoPlano_OFFSET_HORIZONTAL:
                continue
            
            # vamos a correr el plano en z un margen para no contar 
            # dos veces las manzanas del centro.
            z = mapa_profunidad[y, x]
            puntos_con_profundidad.append([x,y,z])

        return puntos_con_profundidad

    def __obtener_plano(self, puntos):
        if len(puntos) < 3:
            raise CantidadPuntosInsuficiente("Hay menos de 3 puntos, no se puede definir el plano")
        
        puntos_a_interpolar = np.array(puntos)

        A = np.c_[puntos_a_interpolar[:, 0], puntos_a_interpolar[:, 1], np.ones(puntos_a_interpolar.shape[0])]
        Z = puntos_a_interpolar[:, 2]

        coeffs, _, _, _ = np.linalg.lstsq(A, Z, rcond=None)
        a, b, c = coeffs
        
        return a, b, c

    def __delante_de_plano(self, x, y, z, a, b, c):
        return a * x + b * y + c + self.FiltradoPlano_MARGEN_PLANO > z

    def __visualizar_plano_en_imagen(self, img, puntos_manzanas, depth_map, a, b, c):
        # Crear una copia de la imagen para dibujar el plano
        img_with_plane = img.copy()

        for y in range(img.shape[0]):
            for x in range(img.shape[1]):
                # Obtener la profundidad desde el mapa de profundidad
                z = depth_map[y, x]

                # Comprobar si el punto está delante del plano.
                esta_delante = self.__delante_de_plano(x, y, z, a, b, c)

                current_color = img_with_plane[y, x]

                # Oscurecer el pixel si cae detras del plano.
                img_with_plane[y, x] = current_color if esta_delante else (int(current_color[0]/2), int(current_color[1]/2), int(current_color[2]/2))

        # Imprimir puntos para las manzanas que estan delante del plano.                
        for [x, y, *rest] in puntos_manzanas:
            # calcular profundidad de la manzana
            z = depth_map[y, x]

            # Comprobar si el punto está delante del plano
            esta_delante = self.__delante_de_plano(x, y, z, a, b, c)

            if esta_delante:
                cv2.circle(img_with_plane, (x, y), 3, (0, 255, 0), -1)
            else:
                cv2.circle(img_with_plane, (x, y), 3, (0, 0, 255), -1)

        return img_with_plane

    def __filtrar_puntos_filas_posteriores(self, puntos, mapa_profundidad):
        return self.filtro_filas_posteriores.filter(None, puntos, None, mapa_profundidad)

    def __pintar_negro_puntos_filtrados(self, timestamp, puntos_originales, puntos_filtrados, img):
        # Crear una copia de la imagen para modificar
        puntos_filtrados_set = set()
        # Crear un conjunto de puntos filtrados para rápida búsqueda
        for _, lista in puntos_filtrados.items():
            for x,y in lista:
                puntos_filtrados_set.add((x,y))

        # Iterar sobre los puntos originales
        for _, puntos in puntos_originales.items():
            for (x, y) in puntos:
                # Si el punto no está en los puntos filtrados, pintarlo de negro
                if (x, y) not in puntos_filtrados_set:
                    cv2.circle(img, (x, y), 3, (0, 0, 0), -1)

    def __filtrar_puntos(self, timestamp, puntos_manzanas, img_original, mapa_profundidad):
        # se obtienen 3 puntos dentro de cada tronco detectado en la imagen    
        puntos_arboles_orig = self.__obtener_puntos_arboles(img_original, timestamp)
        puntos_arboles = {}

        # sacamos los puntos de arboles de filas posteriores
        for mask_id, puntos in puntos_arboles_orig.items():
            puntos_filtrados = self.__filtrar_puntos_filas_posteriores(puntos, mapa_profundidad)
            puntos_arboles[mask_id] = puntos_filtrados

        # pintar de negro los puntos de los arboles filtrados en la imagen 
        if self.config["debug_plano"]:
            self.__pintar_negro_puntos_filtrados(timestamp, puntos_arboles_orig, puntos_arboles, img_original)

        # sacar las keys que tienen listas vacias
        puntos_arboles = {k: v for k, v in puntos_arboles.items() if v}

        # chequeamos la cantidad de troncos detectados
        numero_arboles = len(puntos_arboles.keys())

        if numero_arboles < 2:
            # raise error porque con solo 1 tronco detectado el plano interpolado seria
            # muy poco preciso.
            raise CantidadPuntosInsuficiente("Luego de filtrado los puntos, no quedan 2 arboles")
        
        total_puntos = []
        for puntos_tronco in puntos_arboles.values():
            total_puntos += puntos_tronco

        # agregamos la profunidad de cada punto
        total_puntos = self.__obtener_puntos_con_profunidad(total_puntos, mapa_profundidad)

        # generamos el plano
        a, b, c = self.__obtener_plano(total_puntos)

        if self.config["verbose"]:
            print('plano generado: ')
            print(f"a: {a}, b: {b}, c: {c}")

        if self.FiltradoPlano_GENERAR_IMAGEN_PLANO:
            img_with_plane = self.__visualizar_plano_en_imagen(img_original, puntos_manzanas, mapa_profundidad, a, b, c)
            cv2.imwrite(f"{CWD}/planos/pixeles_filtrados_{timestamp}.png", img_with_plane)

        puntos_filtrados = []

        for [x, y, *rest] in puntos_manzanas:
            if x <= self.FiltradoBase_OFFSET_HORIZONTAL:
                # Evitar franja negra en el mapa de profundidad.
                continue

            # Obtener la profundidad desde el mapa de profundidad
            z = mapa_profundidad[y, x]

            # Comprobar si el punto está delante del plano
            esta_delante = self.__delante_de_plano(x, y, z, a, b, c)

            if esta_delante:
                puntos_filtrados.append([x, y, *rest])

        return puntos_filtrados

    def __preparar_carpetas(self):
        # si la carpeta no existe, la crea, sino se vacia
        if not os.path.exists(f"{CWD}/planos"):
            os.makedirs(f"{CWD}/planos")
        else:
            for file in os.listdir(f"{CWD}/planos"):
                os.remove(f"{CWD}/planos/{file}")

        if not os.path.exists(f"{CWD}/puntos_para_plano"):
            os.makedirs(f"{CWD}/puntos_para_plano")
        else:
            for file in os.listdir(f"{CWD}/puntos_para_plano"):
                os.remove(f"{CWD}/puntos_para_plano/{file}")

        return CWD
