#!/usr/bin/env python3

from . import filtrado_base
from ..plane_method.plane_processing_utils import filtrar_puntos, CantidadPuntosInsuficiente

class FiltradoPlano(filtrado_base.FiltradoBase):
    def filter(self, timestamp, bounding_boxes, img_original, mapa_profundidad):
        working_directory = self.config['working_directory']
        generar_imagen_plano = self.config['generar_imagen_plano']
        filtered_points = []

        try:
            filtered_points = filtrar_puntos(timestamp,bounding_boxes, img_original, mapa_profundidad, working_directory, generar_imagen_plano)
        except CantidadPuntosInsuficiente as e:
            print(f"frame skipped, error: {e}")

        return filtered_points