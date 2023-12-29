#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
import time as Time
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import datetime

bridge = CvBridge()
YOLOv8_model = None

def encontrar_punto_de_corte(lista):
    """
    Encuentra el punto que divide una lista ordenada de enteros en dos clusters,
    minimizando la suma de las varianzas internas de los clusters.
    """
    # Ordenar la lista
    lista_ordenada = sorted(lista)

    # Inicializar el mejor punto de corte y la menor suma de varianzas encontrada
    mejor_punto = None
    menor_varianza_suma = float('inf')

    # Iterar sobre todos los puntos posibles (excepto los extremos)
    for i in range(1, len(lista_ordenada)):
        # Dividir la lista en dos clusters basándonos en el punto actual
        cluster_1 = lista_ordenada[:i]
        cluster_2 = lista_ordenada[i:]

        # Calcular la varianza de cada cluster
        varianza_1 = varianza(cluster_1) if cluster_1 else 0
        varianza_2 = varianza(cluster_2) if cluster_2 else 0

        # Sumar las varianzas
        varianza_suma = varianza_1 + varianza_2

        # Actualizar el mejor punto y la menor varianza suma si es necesario
        if varianza_suma < menor_varianza_suma:
            menor_varianza_suma = varianza_suma
            mejor_punto = lista_ordenada[i - 1]

    return mejor_punto

def varianza(lista):
    """
    Calcula la varianza de una lista de números.
    """
    media = sum(lista) / len(lista)
    varianza = sum((x - media) ** 2 for x in lista) / len(lista)
    return varianza


def process_data(data):
    #rospy.loginfo(data)

    try:
        global_frame = bridge.compressed_imgmsg_to_cv2(data)
        #results = YOLOv8_model.predict(global_frame, stream=True)
        results = YOLOv8_model.track(global_frame, persist=True)
        
        # Visualize the results on the frame
        annotated_frame = results[0].plot()
        print('id of first box in frame')
        print(results[0].boxes[0].id)
        # Save the image
        current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S%f")
        cv2.imwrite("detected_images_YOLOv8/" + current_time + "_detected.jpg", annotated_frame)
    except CvBridgeError as e:
        raise(e)


if __name__ == '__main__':

    bridge = CvBridge()
    YOLOv8_model = YOLO('weights_YOLOv8.pt')

    rospy.init_node('test_node')    
    #sub = rospy.Subscriber("/costar_husky_sensor_config_1/left/image_raw/compressed", CompressedImage, process_data, queue_size = 10)
    sub = rospy.Subscriber("/zed_lateral/zed_lateral/left/image_rect_color/compressed", CompressedImage, process_data, queue_size = 10)
    #sub2 = rospy.Subscriber("/zed_lateral/zed_lateral/imu/data", Imu, process_data)
    #sub3 = rospy.Subscriber("/zed_front/zed_node_front/imu/data", Imu, process_data)

    rospy.loginfo('test node has been started...')
    rospy.spin() #blocks until node is shutdown, Yields activity on other threads