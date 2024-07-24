#!/usr/bin/env python3

from . import filtrado_base
import numpy as np
from sklearn.cluster import KMeans

# importar configuraciones
import configparser
config = configparser.ConfigParser()
config.read('src/apple_fields_ignition/fields_ignition/nodes/config.ini')

OFFSET_HORIZONTAL = config.getint('CONSTANTS', 'offset_horizontal')
MARGEN_PLANO = config.getint('CONSTANTS', 'margen_plano')

class FiltradoFilasPosteriores(filtrado_base.FiltradoBase):
    def filter(self, _1, bounding_boxes, _2, mapa_profundidad):
        return self.__filtrar_puntos(bounding_boxes, mapa_profundidad)

    def obtener_puntos_con_profunidad(self, bounding_boxes, mapa_profundidad):
        return super().obtener_puntos_con_profunidad(bounding_boxes, mapa_profundidad)

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
        puntos_con_profundidad = self.obtener_puntos_con_profunidad(puntos_manzanas, mapa_profundidad)
        puntos_filtrados = self.__filtrar_puntos_threshold(puntos_con_profundidad)

        return self.__quitar_profunidad(puntos_filtrados)
    
    # Encuentra la distancia mas lejana del cluster mas cercano
    # esta distancia sera el threshold para filtrar los puntos.
    def __dar_distancia_maxima_de_cluster_cercano(self, lista):
        lista = np.array(lista)
        
        data = lista.reshape(-1, 1)

        # Inicializar KMeans para 1 cluster, ajustar a los puntos
        # y obtener la inercia
        kmeans_1 = KMeans(n_clusters=1, n_init=10)
        kmeans_1.fit(data)

        inertia_1 = kmeans_1.inertia_

        # Inicializar KMeans para 2 clusters, ajustar a los puntos
        # y obtener la inercia
        kmeans_2 = KMeans(n_clusters=2, n_init=10)
        kmeans_2.fit(data)
        inertia_2 = kmeans_2.inertia_

        if inertia_1 < inertia_2:
            return None
        else:
            # Obtener los centros de los 2 clusters
            cluster_centers = kmeans_2.cluster_centers_
            labels = kmeans_2.labels_

            # Agrupar los elementos por las labels de sus clusters
            cluster_0 = lista[labels == 0].tolist()
            cluster_1 = lista[labels == 1].tolist()

            if cluster_centers[0][0] > cluster_centers[1][0]:
                return min(cluster_0) # las distancias estan invertidas, asi que se toma el minimo.
            else:
                return min(cluster_1)
