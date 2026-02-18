# ROS2 Navigation Pipeline

## Overview
This project implements a mobile robot navigation pipeline using ROS2 and TurtleBot3 in Gazebo simulation.

## How to Run

1. Build:
   colcon build
   source install/setup.bash

2. Launch Gazebo:
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo empty_world.launch.py

3. Run nodes:
   ros2 run path_navigation smoothing_node
   ros2 run path_navigation trajectory_node
   ros2 run path_navigation controller_node
