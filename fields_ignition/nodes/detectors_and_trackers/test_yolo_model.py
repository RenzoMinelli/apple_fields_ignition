# Load the YOLOv8 model
from ultralytics import YOLO
import cv2
from PIL import Image
from math import sqrt
import numpy as np
from sklearn.cluster import KMeans
from datetime import datetime

#OFFSET_HORIZONTAL = 53
OFFSET_HORIZONTAL = 70

class CantidadPuntosInsuficiente(Exception):
    def __init__(self, m):
        self.message = m
    def __str__(self):
        return self.message

def find_clusters(nums, threshold=10):
    if not nums:
        return 0

    # Sort the list of numbers
    sorted_nums = sorted(nums)

    # Initialize the first group
    groups = []
    current_group = [sorted_nums[0]]

    # Iterate through sorted numbers and form groups
    for i in range(1, len(sorted_nums)):
        if sorted_nums[i] - sorted_nums[i - 1] > threshold:
            # If the gap is larger than the threshold, start a new group
            groups.append(min(current_group))
            current_group = [sorted_nums[i]]
        else:
            # Otherwise, add the number to the current group
            current_group.append(sorted_nums[i])

    # Add the last group
    groups.append(min(current_group))
    
    return max(groups)

def obtener_puntos_arboles(timestamp,img, model):
    puntos_arboles = {}

    results = model([img], iou=0.1, conf=0.35,show_conf=True,show_labels=False,show=False)

    # Visualize the results
    for res_id, res in enumerate(results):
        # Plot results image
        im_bgr = res.plot()  # BGR-order numpy array
        im_rgb = Image.fromarray(im_bgr[..., ::-1])  # RGB-order PIL image

        if len(res.masks.xy) < 2:
            raise CantidadPuntosInsuficiente("Hay menos de 2 troncos en la imagen")

        # Show results to screen (in supported environments)
        # im_rgb.show()
        
        for mask_id, mask in enumerate(res.masks.xy):

            # inicializo con una lista vacia
            puntos_arboles[mask_id] = []

            # get the center of each tree            
            x, y, w, h = res.boxes[mask_id].xywh[0].numpy()
            
            #print(f"x: {x}, y: {y}, w: {w}, h: {h}")
            x_center = x
            y_center = y

            #print(f"x_center: {x_center}, y_center: {y_center}")

            cv2.circle(img, (int(x_center), int(y_center)), 3, (255, 255, 255), -1)

            # Inicializa una máscara binaria
            mask_binaria = np.zeros(img.shape[:2], dtype=np.uint8)

            # tuplas de los bordes de la mascara
            data_points = [tuple(elem) for elem in mask]
            data_points = np.array([data_points], dtype=np.int32)  # Convertir a matriz numpy de tipo int32

            # Pinta la máscara binaria
            cv2.fillPoly(mask_binaria, data_points, 1)

            # Encuentra las coordenadas de los píxeles dentro de la máscara
            y, x = np.where(mask_binaria == 1)
            pixeles_tronco_coords = list(zip(x, y))  # Convertir el zip en una lista

            # find the closest point in the mask array to the calculated center
            closest_point = pixeles_tronco_coords[0]
            for point in pixeles_tronco_coords:
                point_x, point_y = point[0], point[1]
                closest_x, closest_y = closest_point[0], closest_point[1]

                if abs(point_x - x_center) + abs(point_y - y_center) < abs(closest_x - x_center) + abs(closest_y - y_center):
                    closest_point = point
                
            # find the furthest point above and below the center
            above_point = closest_point
            below_point = closest_point
            closest_x, closest_y = closest_point[0], closest_point[1]

            for point in pixeles_tronco_coords:
                if point[1] <= closest_y and point[1] <= above_point[1]:
                    above_point = point
                if point[1] >= closest_y and point[1] >= below_point[1]:
                    below_point = point

            #print("ANTES DE AJUSTAR QUARTERS")
            #print(f"centro: {closest_point}, above: {above_point}, below: {below_point}")

            cv2.circle(img, (int(closest_point[0]), int(closest_point[1])), 3, (0, 0, 255), -1)
            cv2.circle(img, (int(above_point[0]), int(above_point[1])), 3, (0, 255, 0), -1)
            cv2.circle(img, (int(below_point[0]), int(below_point[1])), 3, (255, 0, 0), -1)

            # now find the point in the middle of center and above
            diff_x = (closest_x - above_point[0])/2
            diff_y = (closest_y - above_point[1])/2

            above_center_x = above_point[0] + diff_x
            above_center_y = above_point[1] + diff_y

            # now for below
            diff_x = (closest_x - below_point[0])/2
            diff_y = (closest_y - below_point[1])/2

            below_center_x = below_point[0] + diff_x
            below_center_y = below_point[1] + diff_y

            #print("LUEGO DE AJUSTAR")
            #print(f"above: ({above_center_x},{above_center_y}) below: ({below_center_x},{below_center_y})")

            cv2.circle(img, (int(above_center_x), int(above_center_y)), 3, (0, 55, 0), -1)
            cv2.circle(img, (int(below_center_x), int(below_center_y)), 3, (55, 0, 0), -1)

            # search for points in pixeles_tronco_coords
            for point in pixeles_tronco_coords:
                point_x, point_y = point[0], point[1]
                above_x, above_y = above_point[0], above_point[1]
                below_x, below_y = below_point[0], below_point[1]

                if abs(point_x - below_center_x) + abs(point_y - below_center_y) <= abs(below_x - below_center_x) + abs(below_y - below_center_y):
                    below_point = point

                if abs(point_x - above_center_x) + abs(point_y - above_center_y) <= abs(above_x - above_center_x) + abs(above_y - above_center_y):
                    above_point = point


            cv2.circle(img, (int(above_point[0]), int(above_point[1])), 3, (0, 155, 0), -1)
            cv2.circle(img, (int(below_point[0]), int(below_point[1])), 3, (155, 0, 0), -1)

            #cv2.imwrite(f"/home/renzo/catkin_ws/deteccion/marcado_{id_imagen}_{mask_id}.png", img)

            #print("PUNTOS ENCONTRADOS")
            #print(f"centro: {closest_point}, above: {above_point}, below: {below_point}\n\n")


            puntos_arboles[mask_id].append(closest_point)
            puntos_arboles[mask_id].append(above_point)
            puntos_arboles[mask_id].append(below_point)               

        #res.save(filename=f'/home/renzo/catkin_ws/deteccion/result_{timestamp}_{res_id}_.jpg')

    return puntos_arboles

def obtener_puntos_con_profunidad(puntos_arboles, mapa_profunidad):
    puntos_con_profundidad = {}
    for mask_id, puntos in puntos_arboles.items():
        for [x,y] in puntos:

            if x <= OFFSET_HORIZONTAL:
                continue
            
            if mask_id not in puntos_con_profundidad: puntos_con_profundidad[mask_id] = [] 

            z = escalar_profundidad(mapa_profunidad[y, x])
            puntos_con_profundidad[mask_id].append([x,y,z])

    return puntos_con_profundidad

def filtrar_puntos_threshold(puntos_arboles):
    puntos = []
    for mask_id, puntos_de_arbol in puntos_arboles.items():
        puntos += puntos_de_arbol

    threshold = int(find_clusters([p[2] for p in puntos]))

    print(f"threshold hallado: {threshold}")

    puntos_filtrados = {}
    for mask_id, puntos_de_arbol in puntos_arboles.items():
        for p in puntos_de_arbol:
            if p[2] >= threshold:
                if mask_id not in puntos_filtrados: puntos_filtrados[mask_id] = []
                puntos_filtrados[mask_id].append(p)

    return puntos_filtrados

def obtener_plano(puntos):
    if len(puntos) < 3:
        raise CantidadPuntosInsuficiente("Hay menos de 3 puntos, no se puede definir el plano")
    
    print(f"Puntos del plano: {puntos}")
    A = np.array(puntos)
    b = np.ones(len(puntos))
    
    # Solve for the coefficients using least squares
    # The normal equation is (A.T * A) * x = A.T * b
    # We use np.linalg.lstsq to solve this equation which minimizes ||Ax - b||
    # Where x corresponds to the coefficients [a, b, c]
    coeffs, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    
    # The result is the coefficients a, b, c, where we have assumed d = 1
    a, b, c = coeffs
    d = -1  # We assume d = -1 to solve for a, b, c
    
    return a, b, c, d

def delante_de_plano(x, y, z, a, b, c, d):
    return a * x + b * y + c * z + d > 0

def escalar_profundidad(valor_z):
    # Invierte el valor z ya que en el mapa de profundidad, un valor más alto significa más cerca
    #return ((255 - valor_z) / 255) * (MAX_DEPTH - MIN_DEPTH) + MIN_DEPTH
    return valor_z

def visualizar_plano_en_imagen(img, depth_map, a, b, c, d):
    # Crear una copia de la imagen para dibujar el plano
    img_with_plane = img.copy()

    for y in range(img.shape[0]):
        for x in range(img.shape[1]):
            # Asegúrate de que el índice no salga del rango de la imagen

            # Obtener la profundidad desde el mapa de profundidad
            z = depth_map[y, x]
            # Escala la profundidad al rango correcto
            z_scaled = escalar_profundidad(z)

            # Comprobar si el punto está delante del plano
            esta_delante = delante_de_plano(x, y, z_scaled, a, b, c, d)

            current_color = img_with_plane[y, x]
            img_with_plane[y, x] = current_color if esta_delante else (int(current_color[0]/2), int(current_color[1]/2), int(current_color[2]/2))

    return img_with_plane

def filtrar_puntos(timestamp,puntos_manzanas, img_original, mapa_profundidad, model_tronco):
    puntos_arboles = obtener_puntos_arboles(timestamp,img_original, model_tronco)
    puntos_con_profundidad = obtener_puntos_con_profunidad(puntos_arboles, mapa_profundidad)
    puntos_filtrados = filtrar_puntos_threshold(puntos_con_profundidad)

    numero_arboles = len(puntos_filtrados.keys())

    if numero_arboles < 2:
        raise CantidadPuntosInsuficiente("Luego de filtrado los puntos, no quedan 2 arboles")

    total_puntos = []
    for puntos_tronco in puntos_filtrados.values():
        total_puntos += puntos_tronco

    a, b, c, d = obtener_plano(total_puntos)

    img_with_plane = visualizar_plano_en_imagen(img_original, mapa_profundidad, a, b, c, d)
    cv2.imwrite(f"/home/renzo/catkin_ws/deteccion/pixeles_filtrados_{timestamp}.png", img_with_plane)

    puntos_filtrados = []
    puntos_rechazados = []

    for [x, y, apple_id] in puntos_manzanas:
        if x <= OFFSET_HORIZONTAL:
            continue

        # Obtener la profundidad desde el mapa de profundidad
        z = mapa_profundidad[y, x]
        # Escala la profundidad al rango correcto
        z_scaled = escalar_profundidad(z)

        # Comprobar si el punto está delante del plano
        esta_delante = delante_de_plano(x, y, z_scaled, a, b, c, d)

        if esta_delante:
            puntos_filtrados.append([x, y, apple_id])
        else:
            puntos_rechazados.append([x, y, apple_id])

    return puntos_filtrados,puntos_rechazados

if __name__ == "__main__":
    model_tronco = YOLO('/home/renzo/Downloads/OneDrive_1_4-24-2024/simulado_lateral.pt') 
    
    #img_test = cv2.imread("/home/renzo/Desktop/recorrida_ambos_lados/left_rgb_images/41086000000.png")
    #mapa_profundidad = cv2.imread("/home/renzo/Desktop/recorrida_ambos_lados/disparity_images/41086000000.png", cv2.IMREAD_GRAYSCALE)

    img_test = cv2.imread("/home/renzo/catkin_ws/left_rgb_images/70818000000.png")
    mapa_profundidad = cv2.imread("/home/renzo/catkin_ws/disparity_images/70818000000.png", cv2.IMREAD_GRAYSCALE)

    puntos_arboles = obtener_puntos_arboles(img_test, model_tronco)
    puntos_con_profundidad = obtener_puntos_con_profunidad(puntos_arboles, mapa_profundidad)
    puntos_filtrados = filtrar_puntos_threshold(puntos_con_profundidad)

    
    numero_arboles = len(puntos_filtrados.keys())

    if numero_arboles < 2:
        raise CantidadPuntosInsuficiente("Luego de filtrado los puntos, no quedan 2 arboles")

    total_puntos = []
    for puntos in puntos_filtrados.values():
        total_puntos += puntos

    a, b, c, d = obtener_plano(total_puntos)

    print(f"coeficientes plano: {a}, {b}, {c}, {d}")

    # Crear la imagen con el plano visualizado
    img_with_plane = visualizar_plano_en_imagen(img_test, mapa_profundidad, a, b, c, d)
    cv2.imwrite(f"/home/renzo/catkin_ws/deteccion/pixeles_filtrados.png", img_with_plane)
    
    """
    puntos_manzanas_test = [[487,695], [558,682], [427,464], [100,658], [286,706]]

    print(f"Puntos manzanas: {puntos_manzanas_test}")

    filt,rech = filtrar_puntos(puntos_manzanas_test, img_test, mapa_profundidad, model_tronco)

    print(f"filtrados: {filt}, rechazados: {rech}")
    """
