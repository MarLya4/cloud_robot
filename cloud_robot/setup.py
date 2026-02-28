from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'cloud_robot'

# Función para obtener todos los archivos dentro de un directorio
def get_files_recursive(path):
    files = []
    for root, _, filenames in os.walk(path):
        for f in filenames:
            files.append(os.path.join(root, f))
    return files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/robot', glob('robot/*')),
        ('share/' + package_name + '/urdf', get_files_recursive('urdf')),
        ('share/' + package_name + '/urdf/meshes', get_files_recursive('urdf/meshes')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/controllers', glob('controllers/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cloud',
    maintainer_email='cloud@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': []
    },
)
