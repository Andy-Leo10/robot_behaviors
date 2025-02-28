from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
#libraries for configurations
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_name = 'nav2_slam'
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare(pkg_name), 'config', 'slam.yaml']
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation/Gazebo clock'
    )
    declare_rviz_arg = DeclareLaunchArgument('rviz', default_value='True', description='Launch RViz?')
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name), 'rviz', 'config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
        
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_server',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg, 
        slam_params_file_arg, 
        declare_rviz_arg,
        rviz_node,
        slam_node])