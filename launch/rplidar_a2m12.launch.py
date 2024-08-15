import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_a2m12.launch',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': '256000',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'inverted': False
            }]        
        )
    ])