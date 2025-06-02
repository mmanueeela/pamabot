from ultralytics import YOLO
import cv2
import os

# Ruta al modelo YOLO
model_path = '/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/pamabot_vision/pamabot_vision/modelos/best.pt'
model = YOLO(model_path)

# Carpeta con imágenes a testear
images_folder = '/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/pamabot_yolo_dataset/images/train'

# Recorre todas las imágenes
for filename in sorted(os.listdir(images_folder)):
    if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
        image_path = os.path.join(images_folder, filename)
        print(f'\n🖼️ Procesando: {filename}')
        
        img = cv2.imread(image_path)
        if img is None:
            print(f'⚠️ Error leyendo imagen: {image_path}')
            continue

        results = model(img)
        annotated_frame = results[0].plot()

        cv2.imshow("YOLO Batch Test", annotated_frame)
        print(f'🧠 Clases detectadas: {results[0].boxes.cls.tolist()}')
        cv2.waitKey(0)  # Espera a que pulses una tecla

cv2.destroyAllWindows()
print("✅ Prueba completa.")

