# Load the YOLOv8 model
from ultralytics import YOLO
import cv2
from PIL import Image

model = YOLO('/home/pincho/simulado_lateral.pt')
img = cv2.imread("/home/pincho/catkin_ws/right_rgb_images/34023000000.png")

# Con cada imagen que llega:
results = model([img], iou=0.1, conf=0.35,show_conf=True,show_labels=False,show=False)


# Visualize the results
for i, res in enumerate(results):
    # Plot results image
    im_bgr = res.plot()  # BGR-order numpy array
    im_rgb = Image.fromarray(im_bgr[..., ::-1])  # RGB-order PIL image

    # Show results to screen (in supported environments)
    # im_rgb.show()
    
    if len(res.masks) >= 2:
        for i, mask in enumerate(res.masks.xy):
            breakpoint()
            # get the center of each tree            
            x, y, w, h = res.boxes[i].xywh[0].cpu().numpy()
            x_center = round(x + w/2)
            y_center = round(y + h/2)

            # find the closest point in the mask array to the calculated center
            closest = mask[0]
            for point in mask:
                if abs(point[0] - x_center) + abs(point[1] - y_center) < abs(closest[0] - x_center) + abs(closest[1] - y_center):
                    closest = point
                
            # find the furthest point above and below the center
            above = closest
            below = closest
            for point in mask:
                if point[1] < y_center and point[1] > above[1]:
                    above = point
                if point[1] > y_center and point[1] < below[1]:
                    below = point

            # calculate distance between the center and the furthest points
            # TODO calcular distancia entre el centro y los puntos mas lejanos (con norma 1 creo)
            # despues de eso recorrer las mascaras denuevo y buscar la que este mas cerca 
            # del punto medio entre el centro y el punto mas lejano de arriba, y analogamente
            # para el punto mas lejano de abajo. ahi se obtendrian 2 puntos mas. Con esos puntos se arma el plano
            

    else:
        print("menos de 2 arboles.")
        # usar el otro metodo

    # Access the masks within the results
    masks = res.pred[0][:, -1]  # tensor [N, H, W] of class labels

    # Save results to disk
    # r.save(filename=f'/home/renzo/catkin_ws/deteccion/results{i}.jpg')