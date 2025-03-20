import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('fakexomy_description'), 'urdf', 'fakexomy.urdf')
    robot description = 

    return LaunchDescription([
        Node(
            package='fakexomy_description',
            executable='display',
            name='display',
            output='screen'
        )
    ])

