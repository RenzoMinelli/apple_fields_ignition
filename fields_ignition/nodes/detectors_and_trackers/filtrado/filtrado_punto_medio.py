#!/usr/bin/env python3

from . import filtrado_base
import numpy

class FiltradoPuntoMedio(filtrado_base.FiltradoBase):
    def __init__(self, config={}):
        super().__init__(config)
        self.FiltradoPuntoMedio_THRESHOLD_MARGIN = self.imported_config.getfloat('FILTRADO_PUNTO_MEDIO', 'distancia_filtro')
        
    def filter(self, _1, bounding_boxes, _2, mapa_profundidad):
        # se obtienen las profundidades de los bounding boxes
        # [<x, y, z(profundidad), id >,....]
        bounding_boxes_with_depth = self.obtener_puntos_con_profunidad(bounding_boxes, mapa_profundidad)

        # Obtener las profundidades de las bounding boxes filtrando los valores de 255 donde en realidad no hay valor de distancia.
        bounding_boxes_with_depth_filtered = [bb[2] for bb in bounding_boxes_with_depth if bb[2] != 255]

        # Calcula el threshold calculando el punto medio de las distancias.
        threshold = self.__find_middle_point(bounding_boxes_with_depth_filtered)

        # se aplica un margen que podria evitar que se cuenten
        # dos veces las manzanas del centro de la fila.
        threshold += self.FiltradoPuntoMedio_THRESHOLD_MARGIN
        
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

    # Encuentra el punto que divide una lista ordenada de enteros.
    def __find_middle_point(self, lista):
        data = numpy.array(lista).reshape(-1, 1)

        # Encontrar el puntos medio de las distancias dentro de lista
        return numpy.median(data)
