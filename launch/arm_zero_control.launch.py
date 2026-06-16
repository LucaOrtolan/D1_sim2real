import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    executable = os.path.join(
        get_package_prefix('d1_sim2real'),
        'lib', 'd1_sim2real', 'arm_zero_control'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=[executable],
            output='screen',
        )
    ])
