#!/usr/bin/env python3

from . import filtrado_base
import numpy
from sklearn.cluster import KMeans

# importar configuraciones
import configparser
config = configparser.ConfigParser()
config.read('src/apple_fields_ignition/fields_ignition/nodes/config.ini')

FIXED_THRESHOLD = config.getboolean('CONSTANTS', 'fixed_threshold')
THRESHOLD_MARGIN = config.getint('CONSTANTS', 'threshold_margin')

class FiltradoKMeans(filtrado_base.FiltradoBase):
    def filter(self, timestamp, bounding_boxes, img_original, mapa_profundidad):
        # se obtienen las profundidades de los bounding boxes
        bounding_boxes_with_depth = self.__get_depths(bounding_boxes, mapa_profundidad)

        # se obtiene el threshold
        # si FIXED_THRESHOLD es True, se usa un threshold fijo
        # sino, se calcula el threshold con KMeans.
        threshold = 57 if FIXED_THRESHOLD else self.__find_clusters([bb[3] for bb in bounding_boxes_with_depth])

        # se aplica una margen que evita que se cuenten
        # dos veces las manzanas del centro.
        threshold += THRESHOLD_MARGIN
        print(f"with threshold adjusted: {threshold}")

        filtered_bounding_boxes = []

        # nos quedamos con las bounding boxes que tienen una
        # profundidad mayor al threshold. Lo que implica 
        # que son las manzanas que están más cerca de la cámara. 
        for bb in bounding_boxes_with_depth:
            if bb[3] >= threshold:
                filtered_bounding_boxes.append(bb)

        return filtered_bounding_boxes

    # utilizamos el mapa de profunidad para obtener la profundidad de cada bounding box
    def __get_depths(self, bounding_boxes, mapa_profundidad):
        bb_with_depth = []
        for bb in bounding_boxes:
            bb_id = bb[2]
            depth = mapa_profundidad[int(bb[1]), int(bb[0])]

            bb_with_depth.append([bb[0], bb[1], bb_id, depth])
        
        # se retorna una lista de la forma
        # [<x, y, id, profundidad>,....]
        return bb_with_depth
    
    # Encuentra el punto que divide una lista ordenada de enteros en dos clusters,
    # minimizando la suma de las varianzas internas de los clusters.
    def __find_clusters(self, lista):
        lista.sort() # TODO este sort me parece que no es necesario.

        data = numpy.array(lista).reshape(-1, 1)

        # Inicializar KMeans para 1 cluster y obtener la inercia que
        # es una medida de que tan bien se ajustan los datos a los clusters.
        kmeans_1 = KMeans(n_clusters=1, n_init=10)
        kmeans_1.fit(data)
        inertia_1 = kmeans_1.inertia_

        # Inicializar KMeans para 2 clusters y obtener la inercia
        kmeans_2 = KMeans(n_clusters=2, n_init=10)
        kmeans_2.fit(data)
        inertia_2 = kmeans_2.inertia_

        # Se retorna el centro del cluster que tenga el valor de inercia más bajo.
        if inertia_1 < inertia_2:
            # Un cluster es mejor.
            cluster_centers = kmeans_1.cluster_centers_
            return cluster_centers[0][0]
        else:
            # Dos clusters es mejor.
            cluster_centers = kmeans_2.cluster_centers_
            return cluster_centers[0][0] if cluster_centers[0][0] > cluster_centers[1][0] else cluster_centers[1][0] 
