from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    frame_id_config = LaunchConfiguration('frame_id', default='imu_link')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'frame_id',
                default_value='imu_link',
                description='Frame ID for IMU data. Should include namespace for multi-robot.'
            ),

            Node(
                package="ros2_icm20948",
                executable="icm20948_node",
                name="icm20948_node",
                parameters=[
                    {"i2c_address": 0x68},
                    {"frame_id": frame_id_config},
                    {"pub_rate": 10},
                ],
            )
        ]
    )
