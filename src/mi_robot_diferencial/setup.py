import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mi_robot_diferencial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 1. Copiar archivos de lanzamiento
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 2. Copiar archivos URDF / XACRO
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        # 3. Copiar world de la pista y configuración de RViz
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andarraz',
    maintainer_email='andarraz@todo.todo',
    description='Robot Diferencial con Cinematica Propia',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 4. Registrar el nodo ejecutable
            'kinematics_node = mi_robot_diferencial.kinematics_node:main',
        ],
    },
)