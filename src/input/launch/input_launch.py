from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the camera and microphone publisher nodes.
    """
    return LaunchDescription([
        Node(
            package='input',
            executable='camera_publisher',
            name='camera_publisher_node',
            output='screen'
        ),
        Node(
            package='input',
            executable='speech_to_text',
            name='speech_to_text_node',
            output='screen'
        ),
    ])

