# Load the YOLOv8 model
from ultralytics import YOLO
import cv2
from PIL import Image
from math import sqrt
import numpy as np

OFFSET_HORIZONTAL = 53

def obtener_puntos_arboles(img, model):
    puntos_arboles = []

    results = model([img], iou=0.1, conf=0.35,show_conf=True,show_labels=False,show=False)

    # Visualize the results
    for res_id, res in enumerate(results):
        # Plot results image
        im_bgr = res.plot()  # BGR-order numpy array
        im_rgb = Image.fromarray(im_bgr[..., ::-1])  # RGB-order PIL image

        # Show results to screen (in supported environments)
        # im_rgb.show()
        
        if len(res.masks) >= 2:
            for mask_id, mask in enumerate(res.masks.xy):

                # get the center of each tree            
                x, y, w, h = res.boxes[mask_id].xywh[0].numpy()
                print(f"x: {x}, y: {y}, w: {w}, h: {h}")
                x_center = x
                y_center = y

                print(f"x_center: {x_center}, y_center: {y_center}")

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

                print("ANTES DE AJUSTAR QUARTERS")
                print(f"centro: {closest_point}, above: {above_point}, below: {below_point}")

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

                print("LUEGO DE AJUSTAR")
                print(f"above: ({above_center_x},{above_center_y}) below: ({below_center_x},{below_center_y})")

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

                cv2.imwrite(f"/home/renzo/catkin_ws/deteccion/marcado_{mask_id}.png", img)

                print("PUNTOS ENCONTRADOS")
                print(f"centro: {closest_point}, above: {above_point}, below: {below_point}\n\n")

                puntos_arboles.append(closest_point)
                puntos_arboles.append(above_point)
                puntos_arboles.append(below_point)
                

        else:
            print("menos de 2 arboles.")
            # usar el otro metodo

        res.save(filename=f'/home/renzo/catkin_ws/deteccion/result_{res_id}.jpg')

    return puntos_arboles

def obtener_puntos_con_profunidad(puntos, mapa_profunidad):
    puntos_con_profundidad = []
    for p in puntos:
        x,y = p[0], p[1]

        if(x + OFFSET_HORIZONTAL >= mapa_profunidad.shape[1]):
            continue

        z = escalar_profundidad(mapa_profunidad[y, x + OFFSET_HORIZONTAL])
        puntos_con_profundidad.append([x,y,z])

    return puntos_con_profundidad

def obtener_plano(puntos):
    if len(puntos) < 3:
        raise ValueError("At least three points are required to define a plane.")
    
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
    return a * x + b * y + c * z + d < 0

def point_distance_to_plane(x, y, z,a, b, c, d):
    num = abs(a*x + b*y + c*z + d)
    denom = np.sqrt(a**2 + b**2 + c**2)
    distance = num / denom

    # Determine the scale based on the geometry of the space
    # The largest possible distance given the geometry of the points
    max_distance = np.sqrt(1024**2 + 1024**2 + 255**2)
    scaled_distance = (distance / max_distance) * 255

    return 255-scaled_distance

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
            if x + OFFSET_HORIZONTAL >= depth_map.shape[1]:
                continue

            # Obtener la profundidad desde el mapa de profundidad
            z = depth_map[y, x + OFFSET_HORIZONTAL]
            # Escala la profundidad al rango correcto
            z_scaled = escalar_profundidad(z)

            # Comprobar si el punto está delante del plano
            #esta_delante = delante_de_plano(x, y, z_scaled, a, b, c, d)
            distancia_plano = point_distance_to_plane(x, y, z_scaled, a, b, c, d)
            # Pintar el pixel de blanco si está delante del plano, de lo contrario negro
            img_with_plane[y, x] = (distancia_plano, distancia_plano, distancia_plano)
            #img_with_plane[y, x] = (255, 255, 255) if esta_delante else (0, 0, 0)

    return img_with_plane


def filtrar_puntos(puntos, img_original, mapa_profundidad, model_tronco):
    puntos_arboles = obtener_puntos_arboles(img_original, model_tronco)
    puntos_con_profundidad = obtener_puntos_con_profunidad(puntos_arboles, mapa_profundidad)
    a, b, c, d = obtener_plano(puntos_con_profundidad)

    puntos_filtrados = []
    puntos_rechazados = []

    for [x,y] in puntos:
        if x + OFFSET_HORIZONTAL >= mapa_profundidad.shape[1]:
            continue

        # Obtener la profundidad desde el mapa de profundidad
        z = mapa_profundidad[y, x + OFFSET_HORIZONTAL]
        # Escala la profundidad al rango correcto
        z_scaled = escalar_profundidad(z)

        # Comprobar si el punto está delante del plano
        esta_delante = delante_de_plano(x, y, z_scaled, a, b, c, d)

        if esta_delante:
            puntos_filtrados.append([x,y])
        else:
            puntos_rechazados.append([x,y])

    return puntos_filtrados,puntos_rechazados

if __name__ == "__main__":
    model_tronco = YOLO('/home/renzo/Downloads/OneDrive_1_4-24-2024/simulado_lateral.pt') 
    
    #img_test = cv2.imread("/home/renzo/Desktop/recorrida_ambos_lados/left_rgb_images/42835000000.png")
    #mapa_profundidad = cv2.imread("/home/renzo/Desktop/recorrida_ambos_lados/disparity_images/42835000000.png", cv2.IMREAD_GRAYSCALE)

    img_test = cv2.imread("/home/renzo/catkin_ws/left_rgb_images/54979000000.png")
    mapa_profundidad = cv2.imread("/home/renzo/catkin_ws/disparity_images/54979000000.png", cv2.IMREAD_GRAYSCALE)

    puntos_arboles = obtener_puntos_arboles(img_test, model_tronco)
    puntos_con_profundidad = obtener_puntos_con_profunidad(puntos_arboles, mapa_profundidad)
    a, b, c, d = obtener_plano(puntos_con_profundidad)

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
