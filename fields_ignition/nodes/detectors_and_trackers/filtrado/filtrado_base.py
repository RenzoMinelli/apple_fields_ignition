#!/usr/bin/env python3

# importar configuraciones
import configparser

class FiltradoBase: 
    def __init__(self, config={}):
        self.config = config
        self.imported_config = configparser.ConfigParser()
        self.imported_config.read(self.config["config_path"])
        self.FiltradoBase_OFFSET_HORIZONTAL = self.imported_config.getint('CONSTANTS', 'offset_horizontal')

    def filter(self, timestamp, bounding_boxes, img_original, mapa_profundidad):
        raise NotImplementedError
    
    def obtener_puntos_con_profunidad(self, puntos, mapa_profunidad):
        puntos_con_profundidad = []
        for [x, y, *rest] in puntos:

            if x <= self.FiltradoBase_OFFSET_HORIZONTAL:
                continue

            z = mapa_profunidad[y, x]
            puntos_con_profundidad.append([x,y,z,*rest])

        # se retorna una lista de la forma
        # [<x, y, z(profundidad), id >,....]
        return puntos_con_profundidad