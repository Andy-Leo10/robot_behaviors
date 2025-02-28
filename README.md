# BeachBot

- [BeachBot](#beachbot)
  - [General](#general)
    - [GPU setup](#gpu-setup)
    - [Compilation](#compilation)
    - [Meta package compilation](#meta-package-compilation)
    - [Update dependencies](#update-dependencies)
    - [About ros2 controllers](#about-ros2-controllers)
  - [Assets description and meta packages](#assets-description-and-meta-packages)
    - [Worlds](#worlds)
    - [Robots](#robots)
    - [Components](#components)
    - [Meta packages](#meta-packages)
  - [Terminal 1-2: World and Robot](#terminal-1-2-world-and-robot)
    - [Terminal 1: world](#terminal-1-world)
    - [Terminal 2: robot](#terminal-2-robot)
  - [Terminal 3: navigation](#terminal-3-navigation)
  - [Terminal 4: perception](#terminal-4-perception)
  - [Terminal 5: behavior](#terminal-5-behavior)
  - [Terminal 6: other](#terminal-6-other)
  - [References](#references)
    - [Rosbot XL](#rosbot-xl)
    - [Nav2](#nav2)
    - [Behavior Trees](#behavior-trees)
    - [Models for simulation](#models-for-simulation)
    - [Panther](#panther)
    - [Summit XL](#summit-xl)

## General

<!-- NOTE, WARNING, TIP, IMPORTANT, CAUTION -->
> [!TIP] 
> For simplicity, setup the project by running the following commands:
> ```
> ./setup.bash
> ```

<details>
  <summary><b><span style="font-size: 1.3em;">Details and commands</span></b></summary>

  ### GPU setup
  Configure the GPU
  ```
  export DRI_PRIME=1
  export NV_PRIME_RENDER_OFFLOAD=1
  export VK_LAYER_NV_optimus=NVIDIA_only
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  glxinfo | grep "client glx vendor string"
  ```

  ### Compilation
  ```
  colcon build --packages-select <PACKAGE>; source install/setup.bash
  ```

  ### Meta package compilation
  Compile the meta package
  ```
  colcon build --packages-up-to metapkg-world
  colcon build --packages-up-to metapkg-robot
  colcon build --packages-up-to metapkg-component
  ```
  Install dependencies of the meta package
  ```
  rosdep install --from-paths assets/metapkg-component --ignore-src -r -y
  ```

  ### Update dependencies
  ```
  cd /home/asimovo/;
  rosdep init && rosdep update --rosdistro $ROS_DISTRO;
  sudo apt update;
  rosdep install -i --from-path assets --rosdistro $ROS_DISTRO -y
  ```
  <!-- For building Rosbot XL
  ```
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  ``` -->

  ### About ros2 controllers

  <details> <!-- Deactivate a Controller -->
    <summary><b>Deactivate a Controller</b></summary>

  1. **List the controllers to see their current states**:
      ```sh
      ros2 control list_controllers
      ```

  2. **Deactivate the controller**:
      ```sh
      ros2 control set_controller_state <controller_name> inactive
      ```

  3. **Unload the controller** (optional):
      ```sh
      ros2 control unload_controller <controller_name>
      ```
  </details>

  <details> <!-- Activate a Controller -->
    <summary><b>Activate a Controller</b></summary>

 1. **Load the controller if it is not already loaded**:
     ```sh
     ros2 control load_controller <controller_name>
     ```

 2. **Configure the controller** (if needed):
     ```sh
     ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: '<controller_name>'}"
     ```

 3. **Activate the controller**:
     ```sh
     ros2 control set_controller_state <controller_name> active
     ```
  </details>

</details>



## Assets description and meta packages

### Worlds
<details> <!-- empty_world-1.0.0 -->
  <summary><b>empty_world-1.0.0</b></summary>
  Empty world for launching a simulation very fast. It was used for launching the robot and checking its topics and controllers.

  ```
  ros2 launch empty_world launch_world.launch.py
  ```
</details>

<details> <!-- maze_world-1.0.0 -->
  <summary><b>maze_world-1.0.0</b></summary>
  Maze world for testing the robot in a simple environment. There are 2 versions of the maze world, one with a simple maze and the other with actors.

  Currently the maze includes 4 actors, if you want to use the simple maze, you need to modify the world to launch in the launch file.

  ```
  ros2 launch maze_world launch_maze_world.launch.py
  ```
</details>

<details> <!-- beach_world-1.0.0 -->
  <summary><b>beach_world-1.0.0</b></summary>
  Beach world for testing the robot in a complex environment.

  In the following world that is launched, is for editing the world easily and then saving the file of the simulation. After that you can update the official world with the objects and positions you want.

  ```
  source install/setup.bash; ros2 launch beach_world launch_world.launch.py gz_world:=/home/asimovo/assets/beach_world-1.0.0/beach_world/worlds/10_testing.sdf
  ```

  Below is listed the worlds that are available and what contains each one:
  - **10_testing.sdf**: for modifying quickly the world
  - **0_empty.sdf**: an empty world
  - **1_project_beach.sdf**: Ritwik's latest world
  - **3_boulevard_ideal.sdf**: almost oficial, but I had problems when adding roads because the visualization was not correct
  - **4_boulevard_current.sdf**: OFFICIAL for now
  - **9_temp.sdf**: file for trying to add the Tool Bar, but I was not able to find the correct plugin
  - **office.sdf**: not in use, but there are office objects
  - **sun_beach_sand.sdf**: no available anymore, but it was my first world for the project
</details>

<details> <!-- house_world-1.0.1 -->
  <summary><b>house_world-1.0.1</b></summary>
  House world was not used during the development of the project.
</details>

### Robots
<details> <!-- rosbot_xl-1.0.0 -->
  <summary><b>rosbot_xl-1.0.0</b></summary>
  Rosbot XL set of packages cloned from the official repository.
  Just the launch file for launching the robot was created and is basically the same as the one used in the official repository.

  No more changes were made to the robot.

  ```
  ros2 launch rosbot_xl_gazebo asimovo.launch.py
  ```
</details>

### Components
<details> <!-- navigation_rosbot-1.0.0 -->
  <summary><b>navigation_rosbot-1.0.0</b></summary>

  This asset is composed of the following packages:
  - **cartographer_slam**: for mapping the environment
  ```
  ros2 launch cartographer_slam cartographer.launch.py
  ```
  - **nav2_slam**: for mapping the environment better
  ```
  ros2 launch nav2_slam slam_toolbox.launch.py
  ```
  - **map_server**: for providing the map
  ```
  ros2 launch map_server nav2_map_server.launch.py
  ```  
  - **localization_server**: for localizing the robot using amcl
  ```
  ros2 launch localization_server localization.launch.py
  ```  
  - **path_planner_server**: for planning the path. Then it launches other things related to autonomous navigation.
  ```
  ros2 launch path_planner_server path_planner.launch.py
  ros2 launch path_planner_server autonomous_navigation.launch.py
  ```  
  - **nav2_apps**: for using Nav2 API, but it is not used in the project, because we are going to use Behavior Trees.
  ```
  cd /home/asimovo/assets/navigation_rosbot-1.0.0/nav2_apps/scripts
  python3 1_navigate_to_pose.py
  ```
</details>

<details> <!-- perception_rosbot-1.0.0 -->
  <summary><b>perception_rosbot-1.0.0</b></summary>
  This asset is composed of 1 package that uses yolov11 for object detection.

  - **yolov11_ros2**: for detecting objects using yolov11: Only is for object detection, the depth image is not being used, because the image is not recevied correctly and ros2 parameters are not available.
  ```
  ros2 run yolov11_ros2 single_object_detection
  ```
</details>

<details> <!-- behavior_tree_rosbot-1.0.0 -->
  <summary><b>behavior_tree_rosbot-1.0.0</b></summary>

  This asset is composed of the following packages:
  - **behaviortree_cpp**: library for creating the Behavior Trees
  - **behaviortree_ros2**: library for creating the Behavior Trees using the ROS2 wrapper
  - **bt_practice**: for following a [tutorial of Behavior Trees by Sebastian Castro](https://github.com/sea-bass/turtlebot3_behavior_demos), but adapted to ignition and my custom world. The tutorial shows 2 methods of create the Behavior Trees:
    - BehaviorTree.CPP: for creating the Behavior Nodes (recommended, but not the best)
    - py_trees_ros: for creating the Behavior Nodes (not recommended)
  - **bt_beach_bot**: for creating the Behavior Trees using the ROS2 wrapper for the robot project. The main idea is to have many services and topics for the robot to interact with the world and the robot itself.
    ```
    ros2 launch bt_beach_bot main_bt.launch.py
    ```
    This launch file start **Groot2** as a visualization tool for the Behavior Tree xml file(doens't show the status live, because we don't have a license for that), and run `main_bt.cpp` for executing all the registered Behavior Nodes following the Behavior Tree xml file.

</details>

### Meta packages
<details> <!-- metapkg-world -->
  <summary><b>metapk-world</b></summary>
  This meta package is composed of the following packages:

  - empty_world
  - maze_world
  - beach_world
  - house_world

  To compile the meta package, use:
  ```
  colcon build --packages-up-to metapkg-world
  ```
</details>

<details> <!-- metapkg-robot -->
  <summary><b>metapk-robot</b></summary>
  This meta package is composed of the following packages:

  - diff_drive_controller
  - mecanum_drive_controller
  - husarion_gz_worlds
  - imu_sensor_broadcaster
  - micro_ros_msgs
  - micro_ros_agent
  - ros_components_description
  - rosbot_hardware_interfaces
  - rosbot_xl
  - rosbot_xl_bringup
  - rosbot_xl_controller
  - rosbot_xl_description
  - rosbot_xl_gazebo
  - rosbot_xl_utils

  To compile the meta package, use:
  ```
  colcon build --packages-up-to metapkg-robot
  ```
</details>

<details> <!-- metapkg-component -->
  <summary><b>metapk-component</b></summary>
  This meta package is composed of the following packages:

  - cartographer_slam
  - map_server
  - localization_server
  - path_planner_server
  - nav2_apps
  - nav2_slam
  - behaviortree_cpp
  - bt_practice
  - behaviortree_ros2
  - bt_beach_bot

  To compile the meta package, use:
  ```
  colcon build --packages-up-to metapkg-component
  ```
</details>

## Terminal 1-2: World and Robot
Robot in maze, base example.
```
source install/setup.bash; ros2 launch master.launch.py
```
> [!WARNING]  
> The simulation may fail, DON'T WORRY, it's normal (unfortunately).
> You have to relaunch until the simulation doesn't CLOSE.

<details>
   <summary><b>World and Robot</b></summary>

### Terminal 1: world
Launching a simulation from the most basic to the most complete
```
colcon build --packages-up-to metapkg-world
source install/setup.bash; ros2 launch empty_world launch_world.launch.py
source install/setup.bash; ros2 launch maze_world launch_maze_world.launch.py
source install/setup.bash; ros2 launch beach_world launch_world.launch.py
```
Launching a simulation for building, designing and arquitecting the world
```
source install/setup.bash; ros2 launch beach_world launch_world.launch.py gz_world:=/home/asimovo/assets/beach_world-1.0.0/beach_world/worlds/10_testing.sdf
```

### Terminal 2: robot
```
colcon build --packages-up-to metapkg-robot
source install/setup.bash; ros2 launch rosbot_xl_gazebo asimovo.launch.py
```
</details>

## Terminal 3: navigation
If rviz doesn't show the robot
```
export ROS_PACKAGE_PATH=/home/asimovo/assets/rosbot_xl-1.0.0/rosbot_xl_ros/rosbot_xl_description # for RVIZ
```

**Autonomous Navigation**
```
colcon build --packages-select path_planner_server --symlink-install
source install/setup.bash; ros2 launch path_planner_server autonomous_navigation.launch.py
```

<details>
<summary><b>Components of Autonomous Navigation</b></summary>

Mapping using cartographer
```
colcon build --packages-up-to metapkg-component
source install/setup.bash; ros2 launch cartographer_slam cartographer.launch.py
```
Mapping using slam toolbox
```
colcon build --packages-up-to metapkg-component
source install/setup.bash; ros2 launch nav2_slam slam_toolbox.launch.py
```
Save the map: `ros2 run nav2_map_server map_saver_cli -f map`

Providing map
```
colcon build --packages-up-to metapkg-component
source install/setup.bash; ros2 launch map_server nav2_map_server.launch.py
```
Localization
```
colcon build --packages-up-to metapkg-component
source install/setup.bash; ros2 launch localization_server localization.launch.py
```
Planning
```
colcon build --packages-up-to metapkg-component
source install/setup.bash; ros2 launch path_planner_server path_planner.launch.py
```
</details>

## Terminal 4: perception
<details> <!-- Create your virtual environment (1st time) -->
  <summary><b>Create your virtual environment (1st time)</b></summary>

  ```
  cd /home/asimovo/assets/
  apt install python3.10-venv
  python3 -m venv .venv
  source .venv/bin/activate
  pip install torch torchvision torchaudio
  pip install ultralytics
  pip install typeguard
  pip install numpy==1.24.3
  ```
</details>

Using yolov11 (sourcing the virtual environment)
```
source /home/asimovo/assets/.venv/bin/activate
cd /home/asimovo/
colcon build --packages-select yolov11_ros2 --symlink-install
source install/setup.bash; ros2 run yolov11_ros2 single_object_detection
```

<details> <!-- Export the new python path if libraries are not found -->
  <summary><b>Export the new python path if libraries are not found</b></summary>

  ```
  export PYTHONPATH=/home/asimovo/assets/.venv/lib/python3.10/site-packages:$PYTHONPATH
  ```
</details>

## Terminal 5: behavior
Rosbot XL Behavior Trees
```
clear; colcon build --packages-select bt_beach_bot; source install/setup.bash; ros2 run bt_beach_bot main_bt
```
Launch with Groot2 visualization
```
clear; colcon build --packages-select bt_beach_bot; source install/setup.bash; ros2 launch bt_beach_bot main_bt.launch.py
```

## Terminal 6: other
<details>
<summary><b>Commands</b></summary>

Checking
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run tf2_tools view_frames
```
World
```
ign service -s /world/beach_world/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 5000 --req 'reset: {all: true}'
ign model --model <OBJECT> --pose
```
TFs
```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```
</details>

## References
<details>
  <summary><b>Links</b></summary>

### Rosbot XL
- [rosbot_xl_ros](https://github.com/husarion/rosbot_xl_ros/tree/master)
- [rosbot_xl_autonomy](https://github.com/husarion/rosbot-xl-autonomy)

### Nav2
- [Nav2 web](http://nav2.org/)
- [Nav2 docs](https://docs.nav2.org/)
- [Commander API](https://docs.nav2.org/commander_api/index.html)
- [Examples API](https://github.com/ros-navigation/navigation2/tree/main/nav2_simple_commander/nav2_simple_commander)
- [Conversation about Nav2 and outdoors](https://discourse.ros.org/t/how-to-use-nav2-on-outdoor-vehicles/38525/2)
- [Nav2 and GPS](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)

### Behavior Trees
- [Nav2 BT Navigator documentation](https://docs.nav2.org/configuration/packages/configuring-bt-xml.html)
- [BTs and Robotics in AI book](http://arxiv.org/pdf/1709.00084)
- [BehaviorTree.CPP documentation](https://www.behaviortree.dev/)
- Repositories
  - [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
    - [TurtleBot3 Behavior Demos](https://github.com/sea-bass/turtlebot3_behavior_demos)
  - [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2)
    - [ros_behavior_wrappers.md](https://github.com/BehaviorTree/BehaviorTree.ROS2/blob/humble/behaviortree_ros2/ros_behavior_wrappers.md)
    - [bt_topic_sub_node.hpp](https://github.com/BehaviorTree/BehaviorTree.ROS2/blob/humble/behaviortree_ros2/include/behaviortree_ros2/bt_topic_sub_node.hpp)
    - [bt_topic_pub_node.hpp](https://github.com/BehaviorTree/BehaviorTree.ROS2/blob/humble/behaviortree_ros2/include/behaviortree_ros2/bt_topic_pub_node.hpp)
    - [bt_action_node.hpp](https://github.com/BehaviorTree/BehaviorTree.ROS2/blob/humble/behaviortree_ros2/include/behaviortree_ros2/bt_action_node.hpp)
    - [bt_service_node.hpp](https://github.com/BehaviorTree/BehaviorTree.ROS2/blob/humble/behaviortree_ros2/include/behaviortree_ros2/bt_service_node.hpp)
    - [tree_execution_server.md](https://github.com/BehaviorTree/BehaviorTree.ROS2/blob/humble/behaviortree_ros2/tree_execution_server.md)
- [Plugin lib for BT Navigator](https://docs.nav2.org/plugins/index.html#behavior-tree-nodes)
  - [How to write a behavior plugin](https://docs.nav2.org/plugin_tutorials/docs/writing_new_behavior_plugin.html#writing-new-behavior-plugin)
  - [Spin plugin code](https://github.com/ros-navigation/navigation2/blob/add09f60b17f6134416122f8285eba7268d275d7/nav2_behaviors/plugins/spin.cpp#L31)

### Models for simulation
- [free3d](https://free3d.com/3d-models/collada)
- [turbosquid](https://www.turbosquid.com/)
- [cadnav](https://www.cadnav.com/)
- [sketchfab](https://sketchfab.com/)
- [downloadfree3d](https://downloadfree3d.com/)
- [open3dmodel](https://open3dmodel.com/)
- [clara](https://clara.io/)

### Panther
- [panther_ros](https://github.com/husarion/panther_ros)

### Summit XL
- [summit_xl_sim](https://github.com/RobotnikAutomation/summit_xl_sim/tree/humble)

</details>