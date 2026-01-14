import signal
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, EmitEvent
from launch.events.process import SignalProcess
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    first_node = Node(
        package="group31_assignment_2",
        executable="apriltag_detector",
        name="apriltag_detector",
        output="screen"
    )

    second_node = Node(
        package="group31_assignment_2",
        executable="tag_transformer",
        name="tag_transformer",
        output="screen"
    )

    third_node = Node(
        package="group31_assignment_2",
        executable="move_arm",
        name="move_arm",
        output="screen"
    )

    second_node_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=first_node,
            on_exit=[second_node]
        )
    )

    third_node_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=second_node,
            on_exit=[third_node]
        )
    )

    stop_first_node = TimerAction(
        period=5.0,
        actions=[
            EmitEvent(
                event=SignalProcess(
                    signal_number=signal.SIGINT,
                    process_matcher=lambda proc: proc.name.startswith('apriltag_detector')


                )
            )
        ]
    )

    stop_second_node = TimerAction(
        period=10.0,
        actions=[
            EmitEvent(
                event=SignalProcess(
                    signal_number=signal.SIGINT,
                    process_matcher=lambda proc: proc.name.startswith('tag_transformer')


                )
            )
        ]
    )

    return LaunchDescription([
        first_node,
        stop_first_node,
        second_node_handler,
        stop_second_node,
        third_node_handler
    ])