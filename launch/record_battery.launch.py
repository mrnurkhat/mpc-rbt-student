import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1. Запуск твоей ноды с параметрами
        Node(
            package='my_first_node', # Имя твоего пакета из Шага 4
            executable='my_talker',   # Имя исполняемого файла из CMakeLists
            name='student_node_v1',
            parameters=[
                {'min_voltage': 36.0},
                {'max_voltage': 42.0}
            ],
            output='screen'
        ),

        # 2. Запуск записи лога (ros2 bag)
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/battery_voltage', '/battery_percentage'],
            output='screen'
        )
    ])
