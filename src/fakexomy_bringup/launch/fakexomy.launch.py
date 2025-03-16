import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of the current file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Go up three levels to reach Exomy_groundstation
    exomy_root = os.path.abspath(os.path.join(current_dir, '..', '..', '..', '..', '..'))
    # Build the absolute path to r2d2.urdf in exomy_root/fakexomy_bringup/
    urdf_file = os.path.join(exomy_root,'src', 'fakexomy_bringup', 'r2d2.urdf')
    rviz_config = os.path.join(exomy_root,'src', 'fakexomy_bringup', 'core.rviz')
    
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        Node(
            package='depth_processor',
            executable='point_cloud_creator2',
            name='point_cloud_creator2'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])