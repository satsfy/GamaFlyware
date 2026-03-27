from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'mission'
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'px4_config.yaml'
    )

    return LaunchDescription([
        # Start MAVROS node
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[config_file]
        ),
        # # Start your node
        # Node(
        #     package='mission',
        #     executable='detecta_qrcodes_missao',
        #     output='screen',
        # ),
    ])
