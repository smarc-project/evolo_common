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

    succor_command_scheduler_node = Node(
        package='evolo_accoustic_com',
        namespace=robot_ns,
        executable='succor_command_scheduler',
        name='succor_command_scheduler',
        output='screen',
        parameters=[{
            'command_topic':    'waraps/sensor/succor/command',
            'serial_out_topic': 'sensors/succor/to',
            'serial_in_topic':  'sensors/succor/from',
            'feedback_topic':   'waraps/sensor/succor/feedback',
            'tick_rate':        10.0,
        }],
    )

    return LaunchDescription([
        robot_ns_launch_arg,
        succor_command_scheduler_node,
    ])
