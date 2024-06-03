#!/usr/bin/env python3

from . import filtrado_base

class SinFiltrado(filtrado_base.FiltradoBase): 
    def filter(self, timestamp, bounding_boxes, img_original, mapa_profundidad):
        return bounding_boxes