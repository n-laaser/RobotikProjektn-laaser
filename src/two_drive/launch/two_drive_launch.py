from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
            Node(
                package='two_drive',  # Replace with your package name
                executable='nodemaster',  # Replace with the entry point for NodeMaster
                name='nodemaster'
            ),
            Node(
                package='two_drive',
                executable='linefollow',  # Replace with the entry point for LineFollow
                name='linefollow'
            ),
            Node(
                package='two_drive',
                executable='laserturn',  # Replace with the entry point for LaserTurn
                name='laserturn'
            ),
        ])
