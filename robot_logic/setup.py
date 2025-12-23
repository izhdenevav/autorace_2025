from setuptools import find_packages, setup

package_name = 'robot_logic'

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
    maintainer='vlada',
    maintainer_email='vizdeneva@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_detector = robot_logic.line_detector:main',
            'pid_controller = robot_logic.line_follower:main'
        ],
    },
)
