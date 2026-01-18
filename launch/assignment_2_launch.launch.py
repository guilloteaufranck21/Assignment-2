import signal
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, EmitEvent, IncludeLaunchDescription
from launch.events.process import SignalProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    simulation_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ir_launch'),
            'launch',
            'assignment_2.launch.py'
            )
        )
    ) 
    
    first_node = Node(
        package="assignment",
        executable="apriltag_detector",
        name="apriltag_detector",
        output="screen"
    )

    second_node = Node(
        package="assignment",
        executable="tag_transformer",
        name="tag_transformer",
        output="screen"
    )

    third_node = Node(
        package="Assignment-2",
        executable="move_arm",
        name="arm_mover",
        output="screen"
    )

    first_node_handler = TimerAction(
        period=20.0,
        actions=[first_node]
    )

    second_node_handler = TimerAction(
        period=30.0,
        actions=[second_node]
    )

    third_node_handler = TimerAction(
        period=40.0,
        actions=[third_node]
    )

    return LaunchDescription([
        simulation_launch,
        first_node_handler,
        second_node_handler,
        ##third_node_handler
    ])