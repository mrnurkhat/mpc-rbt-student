import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')

    
    return LaunchDescription([
	Node (
		package='my_first_node',
		executable='my_talker',
		name='student_node_v1',
		parameters=[
			{'min_voltage': 36.0},
			{'max_voltage': 42.0}
		]
	),
	ExecuteProcess(
		cmd=['ros2', 'bag', 'record', '/battery_voltage', '/battery_percentage'],
          	output='screen'
	)
    ])
