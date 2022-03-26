from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    talker_node = Node(
        package="sensors",
        executable="speed",
    )

    listener_node = Node(
        package="data_processor",
        executable="speedSub"
    )

    ld.add_action(talker_node)
    ld.add_action(listener_node)

    return ld