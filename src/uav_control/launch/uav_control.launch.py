from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_control',
            executable='uav_control',
            name='uav_control_node',
            output='screen',
            parameters=[
                {'start_x': 0.0}, {'start_y': 0.0}, {'start_z': 2.0},
                {'goal_x': 8.0}, {'goal_y': 8.0}, {'goal_z': 2.0},
            ]
        )
    ])

