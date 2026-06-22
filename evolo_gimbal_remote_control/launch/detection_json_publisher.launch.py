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

    detection_json_publisher_node = Node(
        package='evolo_gimbal_remote_control',
        namespace=robot_ns,
        executable='detection_json_publisher.py',
        name='detection_json_publisher',
        output='screen',
        parameters=[{
            'detections_topic': '/yolo/detections',
            'json_topic':       '/evolo/waraps/sensor/camera/detection',
            'camera_frame':     'evolo/z1_optical_frame',
            'global_frame':     'evolo/odom',
            'image_width':      1920,
            'image_height':     1080,
            'camera_aperture':  57.1,
        }],
    )

    return LaunchDescription([
        robot_ns_launch_arg,
        detection_json_publisher_node,
    ])
