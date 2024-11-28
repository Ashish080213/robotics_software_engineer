from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import time

def generate_launch_description():
    # Turtlesim node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )

    spawn_turtle2 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 1.0, y: 10, theta: 0.0, name: 'turtle2'}\""],
        name='spawn_turtle2',
        shell=True
    )

    spawn_turtle3 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 3.0, y: 7.8, theta: 0.0, name: 'turtle3'}\""],
        name='spawn_turtle3',
        shell=True
    )

    spawn_turtle4 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 7.8, y: 3.0, theta: 0.0, name: 'turtle4'}\""],
        name='spawn_turtle4',
        shell=True
    )

    spawn_turtle5 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 10, y: 1.0, theta: 0.0, name: 'turtle5'}\""],
        name='spawn_turtle5',
        shell=True
    )

    turtle1_move_forward = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/turtle1/teleport_relative', 'turtlesim/srv/TeleportRelative', "\"{linear: 2.0, angular: 0.0}\""],
            name='turtle_move_forward',
            shell=True
        )
    turtle1_move_backward = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/turtle1/teleport_relative', 'turtlesim/srv/TeleportRelative', "\"{linear: -2.0, angular: 0.0}\""],
            name='turtle_move_forward',
            shell=True
        )
    turtle3_move_forward = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/turtle3/teleport_relative', 'turtlesim/srv/TeleportRelative', "\"{linear: 2.0, angular: 0.0}\""],
            name='turtle_move_forward',
            shell=True
        )
    turtle3_move_backward = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/turtle3/teleport_relative', 'turtlesim/srv/TeleportRelative', "\"{linear: -2.0, angular: 0.0}\""],
            name='turtle_move_forward',
            shell=True
        )
    turtle4_move_forward = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/turtle4/teleport_relative', 'turtlesim/srv/TeleportRelative', "\"{linear: 2.0, angular: 0.0}\""],
            name='turtle_move_forward',
            shell=True
        )
    turtle4_move_backward = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/turtle4/teleport_relative', 'turtlesim/srv/TeleportRelative', "\"{linear: -2.0, angular: 0.0}\""],
            name='turtle_move_forward',
            shell=True
        )

    return LaunchDescription([
        turtlesim_node,
        spawn_turtle2,
        spawn_turtle3,
        spawn_turtle4,
        spawn_turtle5,
        turtle1_move_forward,
        turtle1_move_backward,
        turtle3_move_forward,
        turtle3_move_backward,
        turtle4_move_forward,
        turtle4_move_backward
    ])
