from setuptools import find_packages, setup

package_name = 'devkit_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
<<<<<<<< HEAD:src/basekit_driver/setup.py
    maintainer='Agroecology Lab',
    maintainer_email='lab@agroecology.org',
    description='Driver for Open Agbot basekit hardware',
    license='Apache License 2.0',
========
    maintainer='Zauberzeug GmbH',
    maintainer_email='ros@zauberzeug.com',
    description='ROS2 setup for the Feldfreund DevKit',
    license='MIT License',
>>>>>>>> upstream_main/main:devkit_driver/setup.py
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'devkit_driver_node = devkit_driver.devkit_driver_node:main'
        ],
    },
)
