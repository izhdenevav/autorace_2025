from setuptools import setup

package_name = 'start_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='georg',
    maintainer_email='e.maksimov@g.nsu.ru',
    description='YOLO-based cmd_vel controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'start_controller = start_controller.start_controller:main',
        ],
    },
)
