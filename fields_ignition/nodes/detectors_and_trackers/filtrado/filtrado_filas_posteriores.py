#!/usr/bin/env python3

from . import filtrado_base

#OFFSET_HORIZONTAL = 53
OFFSET_HORIZONTAL = 70
MARGEN_PLANO = 1

class FiltradoFilasPosteriores(filtrado_base.FiltradoBase):
    def __init__(self, config):
        super().__init__(config)

    def filter(self, timestamp, bounding_boxes, img_original, mapa_profundidad):
        working_directory = self.config['working_directory']

        return self.__filtrar_puntos(timestamp,bounding_boxes, img_original, mapa_profundidad, working_directory, generar_imagen_plano)
    
    def __find_clusters(self, nums, threshold=10):
        if not nums:
            return 0

        # Sort the list of numbers
        sorted_nums = sorted(nums, reverse=True)
        maximo = sorted_nums[0]

        # Initialize the first group
        puntos_incluidos = [maximo]

        # Iterate through sorted numbers and form groups
        for i in range(1, len(sorted_nums)):
            if abs(maximo - sorted_nums[i]) > threshold:
                break

            puntos_incluidos.append(sorted_nums[i])
        
        return min(puntos_incluidos)

    def __obtener_puntos_con_profunidad(self, puntos, mapa_profunidad):
        puntos_con_profundidad = []
        for [x, y, *rest] in puntos:

            if x <= OFFSET_HORIZONTAL:
                continue

            z = mapa_profunidad[y, x]
            puntos_con_profundidad.append([x,y,z,*rest])

        return puntos_con_profundidad

    def __filtrar_puntos_threshold(self, puntos):
        threshold = int(self.__find_clusters([p[2] for p in puntos]))

        puntos_filtrados = [p for p in puntos if p[2] >= threshold]
        return puntos_filtrados

    def __quitar_profunidad(self, puntos):
        return [[x,y,*rest] for [x,y,z,*rest] in puntos]
    
    def __filtrar_puntos(self, timestamp, puntos_manzanas, img_original, mapa_profundidad, working_directory, generar_imagen_plano=False):
        puntos_con_profundidad = self.__obtener_puntos_con_profunidad(puntos_manzanas, mapa_profundidad)
        puntos_filtrados = self.__filtrar_puntos_threshold(puntos_con_profundidad)

        return self.__quitar_profunidad(puntos_filtrados)
