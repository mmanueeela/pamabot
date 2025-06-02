from ultralytics import YOLO
import cv2
import os

# Ruta al modelo
model_path = '/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/pamabot_vision/pamabot_vision/modelos/best.pt'
model = YOLO(model_path)

# Carpeta con imágenes a testear
images_folder = '/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/pamabot_yolo_dataset/images/train'

# Recorre todas las imágenes
for filename in os.listdir(images_folder):
    if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
        image_path = os.path.join(images_folder, filename)
        print(f'\n🖼️ Procesando: {filename}')
        img = cv2.imread(image_path)

        results = model(img)
        annotated_frame = results[0].plot()

        cv2.imshow('YOLO Detection', annotated_frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        print(f'🧠 Clases detectadas: {results[0].boxes.cls.tolist()}')
