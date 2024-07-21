#!/usr/bin/env python3

from . import filtrado_base

class SinFiltrado(filtrado_base.FiltradoBase): 
    def filter(self, _1, bounding_boxes, _2, _3):
        return bounding_boxes