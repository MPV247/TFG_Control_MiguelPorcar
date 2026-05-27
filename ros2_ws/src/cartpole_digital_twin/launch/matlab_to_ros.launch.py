import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable # <-- Importamos esto para las variables de entorno
from launch_ros.actions import Node

def generate_launch_description():
    # -------------------------------------------------------------------------
    # 1. Configuración de Variables de Entorno (Reemplaza a los "export")
    # -------------------------------------------------------------------------
    # Esto asegura que todo nodo que arranque en este launch use CycloneDDS y el Dominio 0
    os.environ['ROS_DOMAIN_ID'] = '0'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

    # Ruta del paquete
    pkg_dir = get_package_share_directory('cartpole_digital_twin')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'cartpole.urdf')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'cartpole.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # -------------------------------------------------------------------------
        # Forzar las variables en el contexto del Launch por seguridad adicional
        # -------------------------------------------------------------------------
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

        # Descomenta la siguiente línea si quieres los sliders para probar
        # Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui')
    ])
