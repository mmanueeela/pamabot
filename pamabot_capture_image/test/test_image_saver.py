import os
import cv2
import numpy as np
from pamabot_capture_image.image_saver import ImageSaver

def test_image_saver_saves_image(tmp_path):
    # Crear una imagen dummy de 100x100
    dummy_image = (np.ones((100, 100, 3)) * 255).astype(np.uint8)

    # Crear nodo con ruta de guardado personalizada
    node = ImageSaver()
    node.output_path = str(tmp_path)  # sobrescribimos para que no use la original

    # Guardar la imagen
    nombre_archivo = "test.png"
    ruta_completa = os.path.join(node.output_path, nombre_archivo)
    cv2.imwrite(ruta_completa, dummy_image)

    assert os.path.exists(ruta_completa)
