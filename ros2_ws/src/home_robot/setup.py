from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'home_robot'

def collect_data_files(source_dir, install_base):
    data_files = []
    for root, dirs, files in os.walk(source_dir):
        if files:
            install_dir = os.path.join(install_base, os.path.relpath(root, source_dir))
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((install_dir, file_list))
    return data_files

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
    (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
    (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
    (os.path.join('share', package_name, 'models/tb3_burger'), glob(os.path.join('models/tb3_burger', '*'))),
]

# Añadir recursivamente los modelos
data_files.extend(collect_data_files('models', os.path.join('share', package_name, 'models')))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugo',
    maintainer_email='hugo.didi.contacto@gmail.com',
    description='Proyecto Final Robótica de Servicio 2025',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigation_manager = home_robot.navigation_manager:main',
            'overhead_cam_service = home_robot.overhead_cam_service:main',
            'navigation_service = home_robot.navigation_service:main',
            'execute_patrol = home_robot.execute_patrol:main',
            'voice_controller = home_robot.voice_controller:main',
            'lidar_grid_node = home_robot.lidar_grid_node:main',
            'odom_to_tf = home_robot.odom_to_tf:main',
        ],
    },
)
