from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_name')

    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='evolo',
    )

    gimbal_json_publisher_node = Node(
        package='evolo_gimbal_remote_control',
        namespace=robot_ns,
        executable='gimbal_json_publisher.py',
        name='gimbal_json_publisher',
        output='screen',
        parameters=[{
            'feedback_topic': 'gimbal_camera/gimbal_fb',
            'json_topic':     'waraps/sensor/camera/feedback',
        }],
    )

    return LaunchDescription([
        robot_ns_launch_arg,
        gimbal_json_publisher_node,
    ])
