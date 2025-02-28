#!/bin/bash

# Configure the GPU
export DRI_PRIME=1
export NV_PRIME_RENDER_OFFLOAD=1
export VK_LAYER_NV_optimus=NVIDIA_only
export __GLX_VENDOR_LIBRARY_NAME=nvidia
glxinfo | grep "client glx vendor string"

# Meta package compilation
colcon build --packages-up-to metapkg-world
colcon build --packages-up-to metapkg-robot
colcon build --packages-up-to metapkg-component

# Update dependencies
cd /home/asimovo/
rosdep init && rosdep update --rosdistro $ROS_DISTRO
sudo apt update
rosdep install -i --from-path assets --rosdistro $ROS_DISTRO -y

# Source the setup script
source install/setup.bash