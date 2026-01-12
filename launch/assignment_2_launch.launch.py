from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    first_node = Node(
        package="group31_assignment_1",
        executable="move_arm",
        name="arm_mover",
        output="screen"
    )

    return LaunchDescription([
        first_node
    ])