A metapackage in ROS2 is primarily used for grouping related packages together. It doesn't compile the dependent packages automatically; instead, it serves as a convenient way to manage and install a set of related packages. When you build a metapackage, it doesn't trigger the build of its dependencies. You need to build the dependent packages separately.

To build all the packages in your workspace, including the dependencies listed in your metapackage, you should run:

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

This will build all the packages in your workspace, including robot_description and rrbot_description.

If you want to ensure that the dependencies are built when you build the metapackage, you can specify the dependencies explicitly in the colcon build command:

```
cd ~/ros2_ws
colcon build --packages-up-to my_metapkg
source install/setup.bash
```

This command will build my_metapkg and all its dependencies (robot_description and rrbot_description).

Ensuring Binary Dependencies are Installed:
To ensure that the binary dependencies are installed, you can use rosdep to install them:

```
cd ~/ros2_ws
rosdep install --from-paths src/andres-lab/my_metapkg --ignore-src -r -y
```