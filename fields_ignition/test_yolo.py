from ultralytics import YOLO
import cv2
import time
image_to_detect = './IMG-20220407-WA0020.jpeg'
image = cv2.imread(image_to_detect)  # Load the image from file

# calculamos cuando demora en detectar
model = YOLO('weights_grupo.pt')

tiempo_total = 0
for i in range(10):
    tiempo_inicio = time.time()
    results = model(image_to_detect)
    tiempo_fin = time.time()
    tiempo_total += tiempo_fin - tiempo_inicio

print(f"Tiempo de deteccion: {tiempo_total/10}")
result = results[0]

for data_boxes in result.boxes.data.tolist():
    x1, y1, x2, y2, score, class_id = data_boxes

    #print(f"Box: {data_boxes}")
    # draw a bounding box rectangle and label on the image
    cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
    cv2.putText(image, f'{class_id} {score:.2f}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

# Save the image
cv2.imwrite("output_grupo.jpg", image)