import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    bridge = Node(
        package='d1_sim2real',
        executable='d1_bridge_node.py',
        name='d1_bridge_node',
        output='screen',
    )

    zero_position = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[os.path.join(
                    get_package_prefix('d1_sim2real'),
                    'lib', 'd1_sim2real', 'zero_position.py'
                )],
                output='screen',
            )
        ],
    )

    return LaunchDescription([bridge, zero_position])
