from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = Path(get_package_share_directory("mobile_manipulation_bringup"))
    perception_config = str(share_dir / "config" / "perception_3d.yaml")
    grasp_config = str(share_dir / "config" / "grasp_planner.yaml")
    task_config = str(share_dir / "config" / "task_manager.yaml")
    use_mock_detector = LaunchConfiguration("use_mock_detector")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_detector", default_value="true"),
            Node(
                package="perception_3d",
                executable="mock_detector",
                name="mock_detector",
                output="screen",
                parameters=[perception_config],
                condition=IfCondition(use_mock_detector),
            ),
            Node(
                package="perception_3d",
                executable="depth_projector",
                name="depth_projector",
                output="screen",
                parameters=[perception_config],
                remappings=[
                    ("camera_info", "/camera/color/camera_info"),
                    ("depth_image", "/camera/depth/image_raw"),
                    ("detections", "/detections"),
                ],
            ),
            Node(
                package="grasp_planner",
                executable="grasp_pose_planner",
                name="grasp_pose_planner",
                output="screen",
                parameters=[grasp_config],
                remappings=[
                    ("target_point_arm_base", "/target_point_camera"),
                ],
            ),
            Node(
                package="task_manager",
                executable="pick_place_state_machine",
                name="pick_place_state_machine",
                output="screen",
                parameters=[task_config],
            ),
        ]
    )
