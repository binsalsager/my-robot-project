from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ai',
            executable='gemini_node',
            name='gemini_node',
            output='screen'
        ),
        Node(
            package='ai',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[
                {'model_set': 'A'}  # Change 'A' to 'B' to test the new models
            ]
        ),
        Node(
            package='ai',
            executable='logger_node',
            name='logger_node',
            output='screen'
        ),
    ])