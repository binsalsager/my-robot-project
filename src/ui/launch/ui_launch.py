from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the ui package,
    launching both the video display and audio graph nodes.
    """
    return LaunchDescription([
        Node(
            package='ui',
            executable='main_gui',
            name='main_gui_node',
            output='screen'
        ),
        Node(
            package='ui',
            executable='eye_animation',
            name='eye_animation_node',
            output='screen'
        ),
    ])


