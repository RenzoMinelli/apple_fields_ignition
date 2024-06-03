#!/usr/bin/env python3

class FiltradoBase: 
    def __init__(self, config={}):
        self.config = config

    def filter(self, timestamp, bounding_boxes, img_original, mapa_profundidad):
        raise NotImplementedError