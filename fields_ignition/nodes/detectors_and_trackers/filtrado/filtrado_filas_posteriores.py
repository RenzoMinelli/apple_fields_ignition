#!/usr/bin/env python3

from . import filtrado_base

class FiltradoFilasPosteriores(filtrado_base.FiltradoBase):
    def __init__(self, config={}):
        super().__init__(config)
        self.FiltradoFilas_DISTANCIA_FILTRO = self.imported_config.getfloat('FILTRADO_FILAS_POSTERIORES', 'distancia_filtro')

    def filter(self, _1, bounding_boxes, _2, mapa_profundidad):
        return self.__filtrar_puntos(bounding_boxes, mapa_profundidad)

    def obtener_puntos_con_profunidad(self, bounding_boxes, mapa_profundidad):
        return super().obtener_puntos_con_profunidad(bounding_boxes, mapa_profundidad)

    def __filtrar_puntos_threshold(self, puntos):
        return [p for p in puntos if p[2] < self.FiltradoFilas_DISTANCIA_FILTRO]

    def __quitar_profunidad(self, puntos):
        return [[x,y,*rest] for [x,y,z,*rest] in puntos]
    
    def __filtrar_puntos(self, puntos_manzanas, mapa_profundidad):
        puntos_con_profundidad = self.obtener_puntos_con_profunidad(puntos_manzanas, mapa_profundidad)
        puntos_filtrados = self.__filtrar_puntos_threshold(puntos_con_profundidad)

        return self.__quitar_profunidad(puntos_filtrados)