#libraries standards
from launch import LaunchDescription
from launch_ros.actions import Node
#libraries for configurations
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    configuration_basename = 'cartographer.lua'
    declare_rviz_arg = DeclareLaunchArgument('rviz', default_value='True', description='Launch RViz?')
    rviz_config_file = os.path.join(get_package_share_directory('cartographer_slam'), 'rviz', 'config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        declare_rviz_arg,
        rviz_node,
                
        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            remappings=[
                        ('/cmd_vel', '/cmd_vel'),
                        ('/odom', '/odometry/filtered'),
                        ('/imu', '/imu_broadcaster/imu'),
                        ('/scan', '/scan_filtered'),],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
            remappings=[
                        ('/cmd_vel', '/cmd_vel'),
                        ('/odom', '/odometry/filtered'),
                        ('/imu', '/imu_broadcaster/imu'),
                        ('/scan', '/scan_filtered'),],            
        ),
    ]) 
