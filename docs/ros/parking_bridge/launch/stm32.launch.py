from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "stm32_udp_port",
            default_value="24680",
            description="UDP port for board-forwarded STM32 serial chunks.",
        ),
        DeclareLaunchArgument(
            "record_dir",
            default_value="/home/ebaina/parking_sensor_records",
            description="Directory for STM32 raw bytes, metadata, and health records.",
        ),
        DeclareLaunchArgument(
            "enable_recording",
            default_value="true",
            description="Write raw STM32 serial bytes and metadata to record_dir.",
        ),
        DeclareLaunchArgument(
            "analysis_sample_bytes",
            default_value="8192",
            description="Rolling sample size for STM32 protocol-shape analysis.",
        ),
        Node(
            package="parking_bridge",
            executable="stm32_udp_bridge",
            name="parking_stm32_udp_bridge",
            output="screen",
            parameters=[{
                "bind_ip": "0.0.0.0",
                "udp_port": LaunchConfiguration("stm32_udp_port"),
                "record_dir": LaunchConfiguration("record_dir"),
                "enable_recording": LaunchConfiguration("enable_recording"),
                "status_period_sec": 1.0,
                "stale_after_sec": 2.0,
                "analysis_sample_bytes": LaunchConfiguration("analysis_sample_bytes"),
            }],
        ),
    ])
