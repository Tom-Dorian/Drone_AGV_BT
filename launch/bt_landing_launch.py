from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bt_aruco_landing',
            executable='bt_landing_node',
            name='bt_landing',
            output='screen',
            arguments=[
                # optional path to a different tree
                # 'install/bt_aruco_landing/share/bt_aruco_landing/trees/land_tree.xml'
            ]
        )
    ])
