#!/usr/bin/env python3

from . import filtrado_base
import numpy as np
from sklearn.cluster import KMeans

DISTANCIA_FILTRO = 3 # metros

class FiltradoFilasPosteriores(filtrado_base.FiltradoBase):
    def filter(self, _1, bounding_boxes, _2, mapa_profundidad):
        return self.__filtrar_puntos(bounding_boxes, mapa_profundidad)

    def obtener_puntos_con_profunidad(self, bounding_boxes, mapa_profundidad):
        return super().obtener_puntos_con_profunidad(bounding_boxes, mapa_profundidad)

    def __filtrar_puntos_threshold(self, puntos):
        return [p for p in puntos if p[2] < DISTANCIA_FILTRO]

    def __quitar_profunidad(self, puntos):
        return [[x,y,*rest] for [x,y,z,*rest] in puntos]
    
    def __filtrar_puntos(self, puntos_manzanas, mapa_profundidad):
        puntos_con_profundidad = self.obtener_puntos_con_profunidad(puntos_manzanas, mapa_profundidad)
        puntos_filtrados = self.__filtrar_puntos_threshold(puntos_con_profundidad)

        return self.__quitar_profunidad(puntos_filtrados)