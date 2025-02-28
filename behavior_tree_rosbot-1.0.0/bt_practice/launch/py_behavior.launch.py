from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_for_locations = get_package_share_directory("bt_practice")
    default_world_dir = join(pkg_for_locations, "config", "locations.yaml")

    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument(
                "location_file",
                default_value=TextSubstitution(text=default_world_dir),
                description="YAML file name containing map locations in the world.",
            ),
            DeclareLaunchArgument(
                "target_color",
                default_value=TextSubstitution(text="red"),
                description="Target object color (red, green, or blue)",
            ),
            DeclareLaunchArgument(
                "tree_type",
                default_value=TextSubstitution(text="queue"),
                description="Behavior tree type (naive or queue)",
            ),
            DeclareLaunchArgument(
                "enable_vision",
                default_value=TextSubstitution(text="True"),
                description="Enable vision behaviors. If false, do navigation only.",
            ),
            # Main autonomy node
            Node(
                package="bt_practice",
                executable="autonomy_node.py",
                name="autonomy_node_python",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "location_file": LaunchConfiguration("location_file"),
                        "target_color": LaunchConfiguration("target_color"),
                        "tree_type": LaunchConfiguration("tree_type"),
                        "enable_vision": LaunchConfiguration("enable_vision"),
                    }
                ],
            ),
            # Behavior tree visualization: for some reason is not available in APT
            # ExecuteProcess(cmd=["py-trees-tree-viewer", "--no-sandbox"]),
        ]
    )