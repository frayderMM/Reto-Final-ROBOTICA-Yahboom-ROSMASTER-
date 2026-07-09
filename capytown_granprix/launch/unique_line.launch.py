"""Launch del nodo ``unique_line`` (seguimiento de una sola pared con
LiDAR, ver README_unique_line.md).

No depende del resto del paquete Gran Prix (lidar_processor,
state_machine, etc.) -- solo lee ``/scan`` y ``/odom_raw`` y publica
``/cmd_vel``. Pensado para probarse solo, en un pasillo/pista simple,
antes o en paralelo al reto del laberinto.

Argumentos:
    params_file  (ruta)        archivo de parametros a usar
    follow_right (true|false)  seguir la pared derecha (default true)
    follow_left  (true|false)  seguir la pared izquierda (default false)

Ejemplos:
    ros2 launch capytown_granprix unique_line.launch.py
    ros2 launch capytown_granprix unique_line.launch.py follow_right:=false follow_left:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('capytown_granprix')
    default_params_file = os.path.join(pkg_share, 'config', 'unique_line_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Archivo YAML de parametros de unique_line.',
    )
    follow_right_arg = DeclareLaunchArgument(
        'follow_right', default_value='true', description='Seguir la pared derecha.',
    )
    follow_left_arg = DeclareLaunchArgument(
        'follow_left', default_value='false', description='Seguir la pared izquierda.',
    )

    params_file = LaunchConfiguration('params_file')
    follow_right = LaunchConfiguration('follow_right')
    follow_left = LaunchConfiguration('follow_left')

    unique_line_node = Node(
        package='capytown_granprix',
        executable='unique_line_node',
        name='unique_line',
        output='screen',
        parameters=[
            params_file,
            {
                'follow_right': ParameterValue(follow_right, value_type=bool),
                'follow_left': ParameterValue(follow_left, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        params_file_arg,
        follow_right_arg,
        follow_left_arg,
        unique_line_node,
    ])
