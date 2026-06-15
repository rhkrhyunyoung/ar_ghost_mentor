"""
demo_launch.py
────────────────────────────────────────────────────────────
심사 데모용 런치파일 — AR 오버레이 + 대시보드 동시 실행

실행:
    ros2 launch ghost_mentor demo_launch.py \
        ghost_path:=/abs/path/to/screw_tightening_master.json
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ghost_path_arg = DeclareLaunchArgument(
        "ghost_path",
        default_value="",
        description="Ghost 마스터 JSON 파일 절대 경로",
    )
    threshold_arg = DeclareLaunchArgument(
        "threshold",
        default_value="80.0",
        description="성공 판정 일치도 임계값 (%)",
    )
    camera_arg = DeclareLaunchArgument(
        "camera_index",
        default_value="0",
        description="웹캠 인덱스 (기본 0)",
    )

    ar_node = Node(
        package="ghost_mentor",
        executable="ar_overlay_node",
        name="ar_overlay",
        parameters=[{
            "ghost_path":    LaunchConfiguration("ghost_path"),
            "threshold":     LaunchConfiguration("threshold"),
            "camera_index":  LaunchConfiguration("camera_index"),
        }],
        output="screen",
    )

    dashboard_node = Node(
        package="ghost_mentor",
        executable="score_dashboard",
        name="dashboard",
        output="screen",
    )

    return LaunchDescription([
        ghost_path_arg,
        threshold_arg,
        camera_arg,
        ar_node,
        dashboard_node,
    ])
