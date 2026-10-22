"""RViz prediction playback only: never start a driver or controller."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    bringup = Path(get_package_share_directory("zekeep_bringup"))
    moveit = Path(get_package_share_directory("zekeep_moveit_config"))
    # Load the existing model without the ros2_control/mock-hardware launch.
    urdf = xacro.process_file(str(bringup / "description/urdf/sixaxis.urdf")).toxml()
    semantic = (moveit / "config/zekeep.srdf").read_text()
    return LaunchDescription([
        Node(
            package="rviz2", executable="rviz2", name="pi05_preview_rviz",
            output="screen",
            arguments=["-d", str(Path(__file__).with_suffix(".rviz"))],
            parameters=[{
                "robot_description": urdf,
                "robot_description_semantic": semantic,
            }],
            remappings=[("/tf", "/pi05_preview/tf"), ("/tf_static", "/pi05_preview/tf_static")],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="pi05_preview_origin", output="screen",
            arguments=["--frame-id", "pi05_preview_world", "--child-frame-id", "base_link"],
            remappings=[("/tf_static", "/pi05_preview/tf_static")],
        ),
    ])
