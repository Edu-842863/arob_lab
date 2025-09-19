from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    publisher_node = Node(
        package="arob_basic_pkg",
        executable="talker",
        name="my_talker",
    )
    subscriber_node = Node(
        package="arob_basic_pkg",
        executable="listener",
        name="my_listener",
    )
    ld.add_action(publisher_node)
    ld.add_action(subscriber_node)
    return ld