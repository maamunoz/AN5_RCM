"""Modo REAL del AN5/FR5.

Agrupa en un solo launch los tres procesos que hoy se corrian a mano en
terminales separadas (no existia ningun launch file para esto en el
workspace -- fr_ros2 y code no tienen carpeta launch/). No se modifica
ningun archivo de fr_ros2 ni de code: este launch solo ejecuta, sin cambios,
los mismos ejecutables/nodos que ya existian.

Levanta:
  - fr_ros2 / ros2_cmd_server (C++): el driver real. Se conecta por TCP a
    192.168.58.2 (IP hardcodeada en fr_ros2/src/ROS_API.cpp y
    state_feedback.cpp) y hace exit(0) si no logra conectar al arrancar.
    Se lanza con respawn=True, asi que el launch lo reintenta cada 5 s
    hasta que el controlador este accesible (y lo relanza si el proceso
    muere en runtime). El driver ademas reconecta solo sus sockets TCP si
    el controlador cierra la conexion durante la operacion.
  - rosbridge_websocket (puerto 9090), igual que en modo sim.
  - code / publisher_subscriber: igual que en modo sim.

fr_ros2_para.yaml (src/fr_ros2/fr_ros2_para.yaml) NO esta instalado en el
share/ de fr_ros2 (su CMakeLists.txt no lo copia), asi que no se puede
referenciar de forma portable via ament_index. Sus valores por defecto ya
estan hardcodeados en ROS_API.cpp (declare_parameter) y coinciden con el
yaml, asi que el nodo funciona igual sin pasarlo explicitamente. Si
necesitas overridear parametros, pasa fr_ros2_params_file:=/ruta/al/yaml.

NO correr junto con sim.launch.py (choque de /FR_ROS_API_service y de
publishers en nonrt_state_data).
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _make_ros2_cmd_server_node(context, *args, **kwargs):
    params_file = LaunchConfiguration('fr_ros2_params_file').perform(context)
    parameters = [params_file] if params_file else None
    return [Node(
        package='fr_ros2',
        executable='ros2_cmd_server',
        name='FR_ROS_API_node',
        output='screen',
        parameters=parameters,
        # El driver hace exit(0) si no logra conectar al controlador al
        # arrancar; con respawn el launch lo reintenta solo (tanto al inicio
        # como si el proceso muere en runtime) en vez de quedar muerto.
        respawn=True,
        respawn_delay=5.0,
    )]


def generate_launch_description():
    fr_ros2_params_file = DeclareLaunchArgument(
        'fr_ros2_params_file', default_value='',
        description=(
            'Opcional: ruta absoluta a un yaml de parametros para '
            'ros2_cmd_server (p.ej. src/fr_ros2/fr_ros2_para.yaml). No '
            'esta instalado en share/fr_ros2, por eso no hay un default '
            'automatico. Si se deja vacio, se usan los defaults '
            'hardcodeados en ROS_API.cpp (coinciden con el yaml de '
            'fabrica).'))

    matlab_bridge_enabled = DeclareLaunchArgument(
        'matlab_bridge_enabled', default_value='false',
        description=(
            'Si es true, levanta matlab_ik_bridge (puente TCP en '
            'matlab_bridge_port para inverse_kinematics_docker.m). Solo '
            'activar en modo Docker (ver ros2_ws/DOCKER.md); colisiona con '
            'un matlab_ik_node nativo por DDS.'))

    matlab_bridge_port = DeclareLaunchArgument(
        'matlab_bridge_port', default_value='9091',
        description='Puerto TCP de matlab_ik_bridge.')

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch', 'rosbridge_websocket_launch.xml',
            ])
        ),
    )

    publisher_subscriber_node = Node(
        package='code',
        executable='publisher_subscriber',
        name='robot_publisher',
        output='screen',
    )

    matlab_ik_bridge_node = Node(
        package='an5_mock_sim',
        executable='matlab_ik_bridge',
        name='matlab_ik_bridge',
        output='screen',
        parameters=[{'port': LaunchConfiguration('matlab_bridge_port')}],
        condition=IfCondition(LaunchConfiguration('matlab_bridge_enabled')),
    )

    return LaunchDescription([
        fr_ros2_params_file,
        matlab_bridge_enabled,
        matlab_bridge_port,
        rosbridge_launch,
        OpaqueFunction(function=_make_ros2_cmd_server_node),
        publisher_subscriber_node,
        matlab_ik_bridge_node,
    ])
