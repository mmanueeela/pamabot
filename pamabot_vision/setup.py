import os
from setuptools import setup, find_packages

package_name = 'pamabot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['pamabot_vision', 'pamabot_vision.*']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'images'), ['images/mi_codigo.png']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mmanueeelaadmin',
    maintainer_email='manussupv@gmail.com',
    description='Lector de códigos de barra y QR usando OpenCV + ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'barcode_reader = pamabot_vision.barcode_reader:main',
            'imagen_pub = pamabot_vision.publicador_imagen:main',
            'sip_reader = pamabot_vision.sip_reader:main',
            'yolo_detector_node = pamabot_vision.nodes.yolo_detector_node:main',
            'guardar_imagen = pamabot_vision.guardar_imagen:main', 
            'image_saver_timer = pamabot_vision.image_saver_timer:main', 
            'publicador_detectada = pamabot_vision.nodes.publicador_detectada:main',
        ],
    },
)

