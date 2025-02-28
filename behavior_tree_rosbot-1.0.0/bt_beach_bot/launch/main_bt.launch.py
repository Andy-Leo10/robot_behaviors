from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from os import environ
from os.path import join

def generate_launch_description():
    
    # Change this to your Groot2 executable path.
    # This currently defaults to a Groot2 AppImage file in your home directory.
    groot2_executable = "/home/asimovo/squashfs-root/AppRun"

    xml_file_name = "main_bt.xml"
    xml_file_path = join(get_package_share_directory("bt_beach_bot"), 
                         "bt_xml", xml_file_name)

    behavior_tree_node = Node(
        package="bt_beach_bot",
        executable="main_bt",
        name="main_bt",
        output="screen",
        emulate_tty=True,
    )

    groot2_executable = ExecuteProcess(
        cmd=[groot2_executable, "--nosplash", "true", "--file", xml_file_path]
    )

    return LaunchDescription([
        behavior_tree_node, 
        groot2_executable
    ])
