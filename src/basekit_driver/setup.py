import os
from glob import glob
from setuptools import setup

package_name = 'basekit_driver'

setup(
    name=package_name,
    version='0.0.0',
    # Only include directories that actually exist and have __init__.py
    packages=[
        package_name,
        os.path.join(package_name, 'communication'),
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='AgBot Basekit Driver',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basekit_driver_node = basekit_driver.basekit_driver_node:main'
        ],
    },
)
