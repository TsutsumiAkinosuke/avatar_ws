from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="input_apps",
            executable="voice_input",
        ),
        Node(
            package="gemini_ros",
            executable="gemini_ros",
        ),
        Node(
            package="instruction_manager",
            executable="instruction_manager",
        )
    ])