import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

from launch import LaunchDescription
from launch.actions import (
        IncludeLaunchDescription,
)
from launch.actions import ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# new imports
from launch.actions import (DeclareLaunchArgument, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='maze',
        description='Specify the world to launch',
        choices=['empty', 'maze', 'beach']
    )

    ########################### BELOW SIMILAR TO BEFORE ############################
    
    # ros2 launch empty_world launch_world.launch.py
    node_empty_world = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            os.path.join(get_package_share_directory( 'empty_world' ), 'launch', 'launch_world.launch.py')
        ]),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('world'), "' == 'empty'"]))
    )
        
    # ros2 launch maze_world launch_maze_world.launch.py
    node_maze_world = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            os.path.join(get_package_share_directory( 'maze_world' ), 'launch', 'launch_maze_world.launch.py')
        ]),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('world'), "' == 'maze'"]))
    )

    # source install/setup.bash; ros2 launch beach_world launch_world.launch.py
    node_beach_world = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            os.path.join(get_package_share_directory( 'beach_world' ), 'launch', 'launch_world.launch.py')
        ]),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('world'), "' == 'beach'"]))
    )
        
    # ros2 launch rosbot_xl_gazebo asimovo.launch.py
    node_spawn = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            os.path.join(get_package_share_directory( 'rosbot_xl_gazebo' ), 'launch', 'asimovo.launch.py')
        ]),
    )
        
    # for -> ros2 run PKG EXECUTABLE
    # aan_navigation_clients = ExecuteProcess(
    #     cmd=['ros2', 'run', 'aan_navigation_clients', 'field_cover_client'],
    #     output='screen'
    # )    
    
    ########################### DELAYS TO LAUNCHS ############################
    
    return launch.LaunchDescription([
        declare_world_arg,
        node_empty_world,
        node_maze_world,
        node_beach_world,
              
        # node_spawn,
        TimerAction(
            period=5.0,
            actions=[node_spawn]
        ),
      
    ])
    
'''
ros2 launch master.launch.py world:=empty
'''
