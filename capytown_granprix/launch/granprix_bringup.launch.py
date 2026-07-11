"""Launch principal del Reto Final - Gran Prix CapyTown.

Lanza los nodos del reto (lidar_processor, wall_follower,
state_machine, metrics_logger, web_dashboard y, opcionalmente,
stop_sign_detector) con los parametros de ``config/granprix_params.yaml``.

Este launch NO lanza el bringup del robot (driver LiDAR, driver de
motores, microROS, camara): eso lo hace el paquete base del robot
(ver PROPIEDADES_ROBOT.md, ``capytown_esan bringup.launch.py``) y debe
correr antes, por separado.

Argumentos:
    ronda          (1|2)        ronda de la competencia (ver DETALLE RETO 3.md)
    usar_camara    (true|false) activa el nodo de deteccion de PARE/META por camara
    usar_dashboard (true|false) activa el emisor del tablero web de diagnostico
                                 (ver web/dashboard.html, se abre en una laptop
                                 aparte apuntando a la IP del robot)
    params_file    (ruta)       archivo de parametros a usar

Ejemplos:
    ros2 launch capytown_granprix granprix_bringup.launch.py
    ros2 launch capytown_granprix granprix_bringup.launch.py ronda:=2 usar_camara:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('capytown_granprix')
    default_params_file = os.path.join(pkg_share, 'config', 'granprix_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Archivo YAML de parametros del reto.',
    )
    ronda_arg = DeclareLaunchArgument(
        'ronda',
        default_value='1',
        description='1 = ronda de exploracion, 2 = ronda time attack.',
    )
    usar_camara_arg = DeclareLaunchArgument(
        'usar_camara',
        default_value='true',
        description='Activa el nodo de deteccion de PARE/META por camara.',
    )
    usar_dashboard_arg = DeclareLaunchArgument(
        'usar_dashboard',
        default_value='true',
        description='Activa el emisor del tablero web de diagnostico (ver web/dashboard.html).',
    )

    params_file = LaunchConfiguration('params_file')
    ronda = LaunchConfiguration('ronda')
    usar_camara = LaunchConfiguration('usar_camara')
    usar_dashboard = LaunchConfiguration('usar_dashboard')

    lidar_processor_node = Node(
        package='capytown_granprix',
        executable='lidar_processor_node',
        name='lidar_processor',
        output='screen',
        parameters=[params_file],
    )

    wall_follower_node = Node(
        package='capytown_granprix',
        executable='wall_follower_node',
        name='wall_follower',
        output='screen',
        parameters=[params_file],
    )

    state_machine_node = Node(
        package='capytown_granprix',
        executable='state_machine_node',
        name='state_machine',
        output='screen',
        parameters=[params_file, {'usar_camara': ParameterValue(usar_camara, value_type=bool)}],
    )

    stop_sign_detector_node = Node(
        package='capytown_granprix',
        executable='stop_sign_detector_node',
        name='stop_sign_detector',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(usar_camara),
    )

    metrics_logger_node = Node(
        package='capytown_granprix',
        executable='metrics_logger_node',
        name='metrics_logger',
        output='screen',
        parameters=[params_file, {'ronda': ParameterValue(ronda, value_type=int)}],
    )

    web_dashboard_node = Node(
        package='capytown_granprix',
        executable='web_dashboard_node',
        name='web_dashboard',
        output='screen',
        parameters=[params_file, {'usar_camara': ParameterValue(usar_camara, value_type=bool)}],
        condition=IfCondition(usar_dashboard),
    )

    return LaunchDescription([
        params_file_arg,
        ronda_arg,
        usar_camara_arg,
        usar_dashboard_arg,
        lidar_processor_node,
        wall_follower_node,
        state_machine_node,
        stop_sign_detector_node,
        metrics_logger_node,
        web_dashboard_node,
    ])
