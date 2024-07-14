#!/usr/bin/env python3

# importar configuraciones
import configparser
config = configparser.ConfigParser()
config.read('src/apple_fields_ignition/fields_ignition/nodes/config.ini')

OFFSET_HORIZONTAL = config.getint('CONSTANTS', 'offset_horizontal')

class FiltradoBase: 
    def __init__(self, config={}):
        self.config = config

    def filter(self, timestamp, bounding_boxes, img_original, mapa_profundidad):
        raise NotImplementedError
    
    def obtener_puntos_con_profunidad(self, puntos, mapa_profunidad):
        puntos_con_profundidad = []
        for [x, y, bb_id, *rest] in puntos:

            if x <= OFFSET_HORIZONTAL:
                continue

            z = mapa_profunidad[y, x]
            puntos_con_profundidad.append([x,y,z,bb_id,*rest])

        # se retorna una lista de la forma
        # [<x, y, z(profundidad), id >,....]
        return puntos_con_profundidad