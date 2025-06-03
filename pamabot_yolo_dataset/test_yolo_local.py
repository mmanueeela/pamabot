from ultralytics import YOLO
import cv2
import os

# Expandir la ruta al modelo y cargarlo
model_path = os.path.expanduser('~/turtlebot3_ws/src/pamabot/pamabot_vision/pamabot_vision/modelos/best.pt')
model = YOLO(model_path)

# Ruta a la imagen de entrada
img_path = os.path.expanduser('~/turtlebot3_ws/src/pamabot/pamabot_vision/images/ultima_imagen.jpg')
img = cv2.imread(img_path)

# Comprobar si la imagen se cargó bien
if img is None:
    print(f"❌ No se pudo cargar la imagen desde: {img_path}")
    exit(1)

# Ejecutar detección
results = model(img)
boxes = results[0].boxes

# Filtrar detecciones por clase: (0 = aguaoxigenada, 1 = alcohol)
filtered_boxes = boxes[boxes.cls == 1]  # Cambia a == 0 si quieres solo agua oxigenada

# Dibujar las cajas filtradas
img_copy = img.copy()
for box in filtered_boxes:
    xyxy = box.xyxy[0].cpu().numpy().astype(int)
    conf = box.conf[0].item()
    label = f"alcohol {conf:.2f}"  # Corrige la palabra

    cv2.rectangle(img_copy, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (255, 0, 0), 2)
    cv2.putText(img_copy, label, (xyxy[0], xyxy[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

# Guardar y mostrar la imagen
output_path = os.path.expanduser('~/turtlebot3_ws/src/pamabot/pamabot_web/images/detectada.jpg')
cv2.imwrite(output_path, img_copy)
cv2.imshow("YOLO Detección Filtrada", img_copy)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Mostrar las cajas filtradas
print(filtered_boxes)
