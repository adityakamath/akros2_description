from setuptools import setup
import os
from glob import glob

package_name = 'akros2_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf', 'akros2_mecanum'), glob('urdf/akros2_mecanum/*.xacro')),
        (os.path.join('share', package_name, 'meshes', 'akros2_mecanum'), glob('meshes/akros2_mecanum/*.dae')),
        (os.path.join('share', package_name, 'urdf', 'akros2_omni'), glob('urdf/akros2_omni/*.xacro')),
        (os.path.join('share', package_name, 'meshes', 'akros2_omni'), glob('meshes/akros2_omni/*.dae')),
        (os.path.join('share', package_name, 'urdf', 'akros2_diff'), glob('urdf/akros2_diff/*.xacro')),
        (os.path.join('share', package_name, 'meshes', 'akros2_diff'), glob('meshes/akros2_diff/*.dae')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Kamath',
    maintainer_email='adityakamath@live.com',
    description='AKROS2 description files',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
