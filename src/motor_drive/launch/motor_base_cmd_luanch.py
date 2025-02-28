from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    node1 = Node(
        package='motor_drive',
        executable='speed_filtering',
        name='speed_filtering1'
    )
    ld.add_action(node1)
    node2 = Node(
        package='motor_drive',
        executable='motor_drive',
        name='motor_drive1',
        parameters=[
            {'device_id': '/dev/ttyS4'}
            ]
    )
    ld.add_action(node2)
    return ld
