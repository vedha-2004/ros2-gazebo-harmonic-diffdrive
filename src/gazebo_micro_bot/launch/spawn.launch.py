from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('gazebo_micro_bot')
    urdf = os.path.join(pkg_path, 'urdf', 'micro_bot.urdf')

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'micro_bot',
                '-file', urdf,
                '-z', '0.1'
            ],
            output='screen'
        )
    ])
