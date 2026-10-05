import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'an5_mock_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'scripts'),
            glob(os.path.join('scripts', '*.sh'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='martin',
    maintainer_email='mattoc91@gmail.com',
    description=(
        'Modo simulacion (mock) del robot AN5/FR5: reemplaza '
        'ros2_cmd_server sin tocar el stack real ni Unity/rosbridge.'
    ),
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_cmd_server = an5_mock_sim.mock_cmd_server:main',
            'matlab_ik_bridge = an5_mock_sim.matlab_ik_bridge:main',
        ],
    },
)
