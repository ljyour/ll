from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    node1 = Node(
        package='battery_topic',
        executable='battery',
        name='battery1',
        parameters=[
            {'battery_device': '/dev/ttyUSB1'}
        ]
    )
    ld.add_action(node1)


    return ld

if __name__ == '__main__':
    generate_launch_description()