#!/usr/bin/env python3

from . import filtrado_base
import numpy
from sklearn.cluster import KMeans

class FiltradoKMeans(filtrado_base.FiltradoBase):
    def __init__(self, config={}):
        super().__init__(config)
        self.FiltradoKMeans_FIXED_THRESHOLD = self.imported_config.getboolean('CONSTANTS', 'fixed_threshold')
        self.FiltradoKMeans_THRESHOLD_MARGIN = self.imported_config.getint('CONSTANTS', 'threshold_margin')
        
    def filter(self, _1, bounding_boxes, _2, mapa_profundidad):
        # se obtienen las profundidades de los bounding boxes
        # [<x, y, z(profundidad), id >,....]
        bounding_boxes_with_depth = self.obtener_puntos_con_profunidad(bounding_boxes, mapa_profundidad)

        # se obtiene el threshold
        # si FIXED_THRESHOLD es True, se usa un threshold fijo,
        # sino, se calcula el threshold con KMeans.
        # usualmente no es una buena idea utilizar un threshold fijo,
        # de todas formas damos la opcion como forma de experimentación.
        threshold = 57 if self.FiltradoKMeans_FIXED_THRESHOLD else self.__find_clusters([bb[2] for bb in bounding_boxes_with_depth])

        # se aplica un margen que evita que se cuenten
        # dos veces las manzanas del centro de la fila.
        threshold += self.FiltradoKMeans_THRESHOLD_MARGIN
        print(f"Threshold luego de aplicar el margen central: {threshold}")

        filtered_bounding_boxes = []

        # nos quedamos con las bounding boxes que tienen una
        # profundidad mayor al threshold. Lo que implica 
        # que son las manzanas que están más cerca de la cámara.
        for bb in bounding_boxes_with_depth:
            if bb[2] >= threshold:
                filtered_bounding_boxes.append(bb)

        return filtered_bounding_boxes

    def obtener_puntos_con_profunidad(self, bounding_boxes, mapa_profundidad):
        return super().obtener_puntos_con_profunidad(bounding_boxes, mapa_profundidad)

    # Encuentra el punto que divide una lista ordenada de enteros en dos clusters,
    # minimizando la suma de las varianzas internas de los clusters.
    def __find_clusters(self, lista):
        data = numpy.array(lista).reshape(-1, 1)

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
            return cluster_centers[0][0]
        else:
            # Dos clusters es mejor.
            cluster_centers = kmeans_2.cluster_centers_
            return cluster_centers[0][0] if cluster_centers[0][0] > cluster_centers[1][0] else cluster_centers[1][0] 
