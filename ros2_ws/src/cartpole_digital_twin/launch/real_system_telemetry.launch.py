import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable # <-- Necesario para forzar las variables
from launch_ros.actions import Node

def generate_launch_description():
    # -------------------------------------------------------------------------
    # Configuración de Variables de Entorno (Reemplaza a los "export" de terminal)
    # -------------------------------------------------------------------------
    os.environ['ROS_DOMAIN_ID'] = '0'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

    pkg_dir = get_package_share_directory('cartpole_digital_twin')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'cartpole.urdf')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'cartpole.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Forzar las variables en el entorno de ejecución de ROS 2
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),

        # Publicar el URDF del modelo (esqueleto)
        Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Cargar configuración del péndulo en RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),

        # Nodo Driver Serie para conectar con el Péndulo Físico
        Node(
            package='cartpole_digital_twin',
            executable='serial_driver',
            name='serial_driver',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baudrate': 115200}
            ]
        )
    ])