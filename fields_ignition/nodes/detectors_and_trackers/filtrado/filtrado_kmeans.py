#!/usr/bin/env python3

from . import filtrado_base
import numpy
from sklearn.cluster import KMeans

FIXED_THRESHOLD = False
THRESHOLD_MARGIN = 2

class FiltradoKMeans(filtrado_base.FiltradoBase):
    def filter(self, timestamp, bounding_boxes, img_original, mapa_profundidad):
        bounding_boxes_with_depth = self.__get_depths(bounding_boxes, mapa_profundidad)

        threshold = 57 if FIXED_THRESHOLD else self.__find_clusters([bb[3] for bb in bounding_boxes_with_depth]) 
        threshold += THRESHOLD_MARGIN
        print(f"with threshold adjusted: {threshold}")

        filtered_bounding_boxes = []

        for bb in bounding_boxes_with_depth:
            if bb[3] >= threshold:
                filtered_bounding_boxes.append(bb)

        return filtered_bounding_boxes


    def __get_depths(self, bounding_boxes, mapa_profundidad):
        # get the depths of the bounding boxes
        bb_with_depth = []
        for bb in bounding_boxes:
            bb_id = bb[2]
            depth = mapa_profundidad[int(bb[1]), int(bb[0])]

            bb_with_depth.append([bb[0], bb[1], bb_id, depth])
        return bb_with_depth
    

    def __find_clusters(self, lista):
        """
        Encuentra el punto que divide una lista ordenada de enteros en dos clusters,
        minimizando la suma de las varianzas internas de los clusters.
        """
        lista.sort()
        # print(lista)
        lista = numpy.array(lista)
        
        data = lista.reshape(-1, 1)

        # Initialize KMeans model for 1 cluster
        kmeans_1 = KMeans(n_clusters=1, n_init=10)
        kmeans_1.fit(data)
        inertia_1 = kmeans_1.inertia_

        # Initialize KMeans model for 2 clusters
        kmeans_2 = KMeans(n_clusters=2, n_init=10)
        kmeans_2.fit(data)
        inertia_2 = kmeans_2.inertia_

        if inertia_1 < inertia_2:
            # print("Mejor con 1 cluster.")
            cluster_centers = kmeans_1.cluster_centers_
            # print(f"cluster center: {cluster_centers[0][0]}")
            return cluster_centers[0][0]
        else:
            # print("Mejor con 2 clusters.")
            # Get the cluster centers for 2 clusters
            cluster_centers = kmeans_2.cluster_centers_
            # print('cluster centers: ', '0:', cluster_centers[0][0], '1: ', cluster_centers[1][0])
            return cluster_centers[0][0] if cluster_centers[0][0] > cluster_centers[1][0] else cluster_centers[1][0]
