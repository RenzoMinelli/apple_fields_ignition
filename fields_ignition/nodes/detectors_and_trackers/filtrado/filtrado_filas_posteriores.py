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
        threshold = self.__find_division_point([p[2] for p in puntos])

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

    def __find_division_point(self, lista, umbral_distancia=10):
        """
        Encuentra el punto de división óptimo en una lista de valores para dividirla en dos conjuntos,
        si los conjuntos resultantes tienen una diferencia significativa.
        """
        lista.sort()
        
        # Verificar si la diferencia máxima es menor que el umbral
        if lista[-1] - lista[0] < umbral_distancia:
            return None  # No dividir si todos los puntos están cerca
        
        min_variance_sum = float('inf')
        division_point = lista[0]

        for i in range(1, len(lista)):
            left_cluster = lista[:i]
            right_cluster = lista[i:]

            left_variance = np.var(left_cluster) if len(left_cluster) > 1 else 0
            right_variance = np.var(right_cluster) if len(right_cluster) > 1 else 0

            variance_sum = len(left_cluster) * left_variance + len(right_cluster) * right_variance

            if variance_sum < min_variance_sum:
                min_variance_sum = variance_sum
                division_point = lista[i - 1]

        return division_point
