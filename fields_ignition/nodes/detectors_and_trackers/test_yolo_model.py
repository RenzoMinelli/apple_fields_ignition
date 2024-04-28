# Load the YOLOv8 model
from ultralytics import YOLO
import cv2
from PIL import Image
from math import sqrt
import numpy as np

model = YOLO('/home/renzo/Downloads/OneDrive_1_4-24-2024/simulado_lateral.pt')
img = cv2.imread("/home/renzo/catkin_ws/right_rgb_images/54979000000.png")

# Con cada imagen que llega:
results = model([img], iou=0.1, conf=0.35,show_conf=True,show_labels=False,show=False)

puntos_arboles = []

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

            print(f"pixeles_tronco_coords: {pixeles_tronco_coords}")

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