from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_logic',
            executable='line_detector',
            name='line_detector',
            output='screen'
        ),
        Node(
            package='robot_logic',
            executable='pid_controller',
            name='pid_controller',
            output='screen',
            parameters=[{'kp': 0.6, 'ki': 0.0, 'kd': 0.05, 'speed': 0.2}]
        ),
        Node(
            package='start_controller',
            executable='start_controller',
            name='start_controller',
            parameters=[
                {'image_topic': '/color/image'},
                {'target_class': 'green circle'},
                {'conf_threshold': 0.96},
            ]
        ),
        Node(
            package='conjunction_controller',
            executable='conjunction_controller',
            name='conjunction_controller',
            parameters=[
                {'image_topic': '/color/image'},
            ]
        ),
    ])