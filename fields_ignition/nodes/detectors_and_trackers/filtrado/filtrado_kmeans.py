#!/usr/bin/env python3

from . import filtrado_base
import numpy
from sklearn.cluster import KMeans

class FiltradoKMeans(filtrado_base.FiltradoBase):
    def __init__(self, config={}):
        super().__init__(config)
        self.FiltradoKMeans_THRESHOLD_MARGIN = self.imported_config.getfloat('CONSTANTS', 'threshold_margin')
        
    def filter(self, _1, bounding_boxes, _2, mapa_profundidad):
        # se obtienen las profundidades de los bounding boxes
        # [<x, y, z(profundidad), id >,....]
        bounding_boxes_with_depth = self.obtener_puntos_con_profunidad(bounding_boxes, mapa_profundidad)

        # Obtener las profundidades de las bounding boxes filtrando los valores de 255 donde en realidad no hay valor de distancia.
        bounding_boxes_with_depth_filtered = [bb[2] for bb in bounding_boxes_with_depth if bb[2] != 255]

        # Calcula el threshold con KMeans.
        threshold = self.__find_clusters(bounding_boxes_with_depth_filtered)

        # se aplica un margen que evita que se cuenten
        # dos veces las manzanas del centro de la fila.
        threshold -= self.FiltradoKMeans_THRESHOLD_MARGIN
        
        print('threshold:', threshold)

        if self.config["verbose"]:
            print(f"Threshold luego de aplicar el margen central: {threshold}")

        filtered_bounding_boxes = []

        # nos quedamos con las bounding boxes que tienen una
        # profundidad mayor al threshold. Lo que implica 
        # que son las manzanas que están más cerca de la cámara.
        for [x,y,z,*rest] in bounding_boxes_with_depth:
            if z < threshold:
                filtered_bounding_boxes.append([x,y,*rest])

        return filtered_bounding_boxes

    def obtener_puntos_con_profunidad(self, bounding_boxes, mapa_profundidad):
        return super().obtener_puntos_con_profunidad(bounding_boxes, mapa_profundidad)

    # Encuentra el punto que divide una lista ordenada de enteros en dos clusters,
    # minimizando la suma de las varianzas internas de los clusters.
    def __find_clusters(self, lista):
        data = numpy.array(lista).reshape(-1, 1)

        # casos en donde se rompe kmeans con 2 clusters
        largo = len(data)
        if largo == 0:
            # si data esta vacio no se va a usar el threshold asi que no importa
            return 0 
        elif largo == 1:
            return data[0]

        # Inicializar KMeans para 1 cluster y obtener la inercia, que
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
            print('return 1 cluster: ', cluster_centers[0][0])
            return cluster_centers[0][0]
        else:
            # Dos clusters es mejor.
            cluster_centers = kmeans_2.cluster_centers_
            print('return 2 clusters: ', cluster_centers[0][0], " | ", cluster_centers[1][0])
            return cluster_centers[0][0] if cluster_centers[0][0] < cluster_centers[1][0] else cluster_centers[1][0]
