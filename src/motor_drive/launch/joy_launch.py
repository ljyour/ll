from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    node1 = Node(
        package='joy',
        executable='joy_node',
        name='joy1'
    )
    ld.add_action(node1)

    node2 = Node(
        package='motor_drive',
        executable='joy_cmd',
        name='joy_cmd1'
    )
    ld.add_action(node2)

    motor_dvice_launch_dir = os.path.join(
        get_package_share_directory('motor_drive'),
        'launch'
    )
    base_launch_path = os.path.join(motor_dvice_launch_dir,'motor_base_cmd_luanch.py')
    include_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path)
    ) 
    ld.add_action(include_base_launch)
    return ld

