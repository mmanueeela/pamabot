from ultralytics import YOLO
import cv2

# Ruta al modelo entrenado (ajusta la ruta si hace falta)
model = YOLO('/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/pamabot_vision/pamabot_vision/modelos/best.pt')

# Ruta a la imagen del dataset que quieras probar
img = cv2.imread('/home/mmanueeelaadmin/Escritorio/prueba.jpg')

# Ejecutar la detección
results = model(img)

# Mostrar resultados
annotated_frame = results[0].plot()
cv2.imshow("YOLO Local Test", annotated_frame)
cv2.imwrite("/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/pamabot_web/images/detectada.jpg", annotated_frame)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Mostrar coordenadas y clases detectadas
print(results[0].boxes)

