# Copyright 2025 PAMABOT

import rclpy
from pamabot_vision.publicador_imagen import ImagenPublisher

def test_imagen_publisher_loads_and_publishes():
    rclpy.init()
    node = ImagenPublisher()

    # Verificar que cargó la imagen
    assert node.imagen is not None, "La imagen no se cargó correctamente."

    # Llamar manualmente al callback de publicación
    try:
        node.timer_callback()
    except Exception as e:
        assert False, f"Error al publicar la imagen: {e}"

    rclpy.shutdown()
