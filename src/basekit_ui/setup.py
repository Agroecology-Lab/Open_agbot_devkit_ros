from setuptools import setup

package_name = 'basekit_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='sam@todo.todo',
    description='AgBot User Interface',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basekit_ui_node = basekit_ui.basekit_ui_node:main'
        ],
    },
)
