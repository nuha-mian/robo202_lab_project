import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tricycle_camera_pkg')
    tricycle_urdf_path = os.path.join(pkg_share, 'urdf', 'tricycle.urdf')
    obstacles_urdf_path = os.path.join(pkg_share, 'urdf', 'obstacles.urdf')
        
    return LaunchDescription([
        # Launch Gazebo Classic
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn tricycle robot from URDF file
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_tricycle',
            output='screen',
            arguments=[
                '-file', tricycle_urdf_path,
                '-entity', 'tricycle',
                '-x', '0',
                '-y', '0',
                '-z', '0.1'
            ],
        ),

        # Spawn obstacles
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_obstacles',
            output='screen',
            arguments=[
                '-file', obstacles_urdf_path,
                '-entity', 'obstacles',
                '-x', '0.8',
                '-y', '0',
                '-z', '0.15'
            ],
        ),

        # Image Capture Node
        Node(
            package='tricycle_camera_pkg',
            executable='image_capture',
            name='image_capture',
            output='screen',
        )
    ])
