from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches both the web server and the Arduino bridge for remote control.
    """
    return LaunchDescription([
        Node(
            package='remote_control',
            executable='web_server',
            name='web_server_node',
            output='screen'
        ),
        Node(
            package='remote_control',
            executable='arduino_bridge',
            name='arduino_bridge_node',
            output='screen'
        ),
    ])

    

