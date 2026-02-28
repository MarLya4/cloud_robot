from setuptools import find_packages, setup
from glob import glob
package_name = 'cloud_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # robot folder (si tienes algo ahí)
        ('share/' + package_name + '/robot', glob('robot/*')),
        # URDF/XACRO y archivos relacionados
        ('share/' + package_name + '/urdf', [
            'urdf/URDF_DIRECCION_X.xacro',
            'urdf/materials.xacro',
            'urdf/ros2_control.xacro',
            'urdf/URDF_DIRECCION_X.gazebo',
            'urdf/URDF_DIRECCION_X.trans',
        ]),
        # meshes (subcarpetas incluidas)
        ('share/' + package_name + '/urdf/meshes', glob('urdf/meshes/**/*', recursive=True)),
        # launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # config files
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cloud',
    maintainer_email='cloud@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ]
    },
)
