from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Turtlesim node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )

    # Custom task2 node
    task2_node = Node(
        package='module_2_assignment',
        executable='task2',
        name='task1_turtlesim'
    )

    return LaunchDescription([
        turtlesim_node,
        task2_node
    ])
