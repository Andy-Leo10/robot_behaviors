from ament_index_python.packages import get_package_share_directory
from os.path import join
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch_ros.actions import SetParameter
from launch import LaunchDescription
###################################### Setting environment variables for Gazebo
import os
from ament_index_python.packages import get_package_prefix
###################################### ---------------------------------

def generate_launch_description():
    ###################################### Setting environment variables for Gazebo
    package_description = 'empty_world'  # Replace with your package name
    package_directory = get_package_share_directory(package_description)
    # Set the Path to Robot Mesh Models for Loading in Gazebo Sim #
    # NOTE: Do this BEFORE launching Gazebo Sim #
    install_dir_path = (get_package_prefix(package_description) + "/share")
    robot_meshes_path = os.path.join(package_directory, "meshes")
    pkg_models_path = os.path.join(package_directory, "worlds/models")
    gazebo_resource_paths = [install_dir_path, robot_meshes_path, pkg_models_path]
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["IGN_GAZEBO_RESOURCE_PATH"]:
                os.environ["IGN_GAZEBO_RESOURCE_PATH"] += (':' + resource_path)
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (':'.join(gazebo_resource_paths))   
    print(os.environ["IGN_GAZEBO_RESOURCE_PATH"])
    ###################################### ---------------------------------
    
    pkg_share_dir = FindPackageShare(package='empty_world').find('empty_world') + '/'
              
    cloud = LaunchConfiguration('cloud')
            
    gazebo_launch_node_cloud_gz = IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
                ]),
                launch_arguments = {'gz_args': ['-r -v 4 -s --headless-rendering ' + pkg_share_dir + 'worlds/empty.sdf']}.items(),
                condition=IfCondition(cloud)
            )
    gazebo_launch_node_local_gz = IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
                ]),
                launch_arguments = {'gz_args': ['-r -v 4 ' + pkg_share_dir + 'worlds/empty.sdf']}.items(),
                condition=UnlessCondition(cloud)
            )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument(name='cloud', default_value='False', description='Sets the rendering based on whether the simulation runs locally or on the cloud'),
        gazebo_launch_node_cloud_gz,
        gazebo_launch_node_local_gz
    ])
