from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('gazebo_micro_bot')
    urdf = os.path.join(pkg_path, 'urdf', 'micro_bot.urdf')

    return LaunchDescription([

        # 1. Start Gazebo Harmonic
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        # 2. Spawn the robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'micro_bot',
                '-file', urdf,
                '-z', '0.1'
            ],
            output='screen'
        ),

        # 3. Bridge cmd_vel (ROS -> Gazebo)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            output='screen'
        ),

        # 4. Bridge TF (Gazebo -> ROS /tf)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
            ],
            parameters=[{
                'qos_overrides./tf.publisher.durability': 'volatile',
                'qos_overrides./tf.publisher.reliability': 'reliable'
            }],
            output='screen'
        ),

        # 5. Bridge static TF (Gazebo -> ROS /tf_static)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
            ],
            parameters=[{
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
                'qos_overrides./tf_static.publisher.reliability': 'reliable'
            }],
            output='screen'
        )

    ])
