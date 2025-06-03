from ultralytics import YOLO
import cv2

# Ruta al modelo entrenado
model = YOLO('/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/pamabot_vision/pamabot_vision/modelos/best.pt')

# Leer imagen
img = cv2.imread('/home/mmanueeelaadmin/Escritorio/prueba.jpg')

# Ejecutar detección
results = model(img)
boxes = results[0].boxes

# Filtrar detecciones por clase (0 = aguaoxigenada)
filtered_boxes = boxes[boxes.cls == 1]
# filtered_boxes = boxes[boxes.cls == 1]

# Dibujar solo las cajas filtradas sobre la imagen original
img_copy = img.copy()
for box in filtered_boxes:
    xyxy = box.xyxy[0].cpu().numpy().astype(int)
    conf = box.conf[0].item()
    #label = f"aguaoxigenada {conf:.2f}"
    label = f"alchol {conf:.2f}"
    cv2.rectangle(img_copy, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (255, 0, 0), 2)
    cv2.putText(img_copy, label, (xyxy[0], xyxy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

# Mostrar y guardar resultado
cv2.imshow("YOLO Filtrado", img_copy)
cv2.imwrite("/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/pamabot_web/images/detectada.jpg", img_copy)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Mostrar las cajas filtradas
print(filtered_boxes)
