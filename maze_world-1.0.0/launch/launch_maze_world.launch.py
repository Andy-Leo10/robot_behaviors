from ament_index_python.packages import get_package_share_directory
from os.path import join
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch_ros.actions import SetParameter
from launch import LaunchDescription

def generate_launch_description():
    pkg_share_dir = FindPackageShare(package='maze_world').find('maze_world') + '/'
              
    cloud = LaunchConfiguration('cloud')
            
    gazebo_launch_node_cloud_gz = IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
                ]),
                launch_arguments = {'gz_args': ['-r -v 4 -s --headless-rendering ' + pkg_share_dir + 'worlds/maze_actors.sdf']}.items(),
                condition=IfCondition(cloud)
            )
    gazebo_launch_node_local_gz = IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
                ]),
                launch_arguments = {'gz_args': ['-r -v 4 ' + pkg_share_dir + 'worlds/maze_actors.sdf']}.items(),
                condition=UnlessCondition(cloud)
            )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument(name='cloud', default_value='False', description='Sets the rendering based on whether the simulation runs locally or on the cloud'),
        gazebo_launch_node_cloud_gz,
        gazebo_launch_node_local_gz
    ])
