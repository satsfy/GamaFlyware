import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_path = os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', 'default.sdf')
    model_path = os.path.join(get_package_share_directory('your_drone_package'), 'models', 'x500', 'x500.sdf')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_path, description='Gazebo world file'),
        DeclareLaunchArgument('model', default_value=model_path, description='Drone model SDF file'),
        
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=[LaunchConfiguration('world')],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'x500', '-file', LaunchConfiguration('model'), '-x', '0', '-y', '0', '-z', '1'],
            output='screen'
        ),
    ])
