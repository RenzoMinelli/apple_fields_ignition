#!/usr/bin/env python3

from . import filtrado_base
import numpy as np
from sklearn.cluster import KMeans

#OFFSET_HORIZONTAL = 53
OFFSET_HORIZONTAL = 70
MARGEN_PLANO = 1

class FiltradoFilasPosteriores(filtrado_base.FiltradoBase):
    def __init__(self, config):
        super().__init__(config)

    def filter(self, timestamp, bounding_boxes, img_original, mapa_profundidad):
        return self.__filtrar_puntos(bounding_boxes, mapa_profundidad)

    def __obtener_puntos_con_profunidad(self, puntos, mapa_profunidad):
        puntos_con_profundidad = []
        for [x, y, *rest] in puntos:

            if x <= OFFSET_HORIZONTAL:
                continue

            z = mapa_profunidad[y, x]
            puntos_con_profundidad.append([x,y,z,*rest])

        return puntos_con_profundidad

    def __filtrar_puntos_threshold(self, puntos):
        threshold = self.__dar_distancia_maxima_de_cluster_cercano([p[2] for p in puntos])

        puntos_filtrados = []
        if threshold:
            puntos_filtrados = [p for p in puntos if p[2] >= threshold]
        else:
            puntos_filtrados = puntos

        return puntos_filtrados

    def __quitar_profunidad(self, puntos):
        return [[x,y,*rest] for [x,y,z,*rest] in puntos]
    
    def __filtrar_puntos(self, puntos_manzanas, mapa_profundidad):
        puntos_con_profundidad = self.__obtener_puntos_con_profunidad(puntos_manzanas, mapa_profundidad)
        puntos_filtrados = self.__filtrar_puntos_threshold(puntos_con_profundidad)

        return self.__quitar_profunidad(puntos_filtrados)
    
    def __dar_distancia_maxima_de_cluster_cercano(self, lista):
        """
        Encuentra la distancia mas lejana del cluster mas cercano para hacer como filtro
        """
        lista.sort()
        lista = np.array(lista)
        
        data = lista.reshape(-1, 1)

        # Initialize KMeans model for 1 cluster
        kmeans_1 = KMeans(n_clusters=1, n_init=10)
        kmeans_1.fit(data)

        inertia_1 = kmeans_1.inertia_

        # Initialize KMeans model for 2 clusters
        kmeans_2 = KMeans(n_clusters=2, n_init=10)
        kmeans_2.fit(data)
        inertia_2 = kmeans_2.inertia_

        if inertia_1 < inertia_2:
            cluster_centers = kmeans_1.cluster_centers_
            return lista[0]
        else:
            # Get the cluster centers for 2 clusters
            cluster_centers = kmeans_2.cluster_centers_
            labels = kmeans_2.labels_

            # Group elements by their cluster labels
            cluster_0 = lista[labels == 0].tolist()
            cluster_1 = lista[labels == 1].tolist()

            if cluster_centers[0][0] > cluster_centers[1][0]:
                return min(cluster_0)
            else:
                return min(cluster_1)
