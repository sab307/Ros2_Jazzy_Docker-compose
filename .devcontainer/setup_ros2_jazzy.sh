#!/bin/bash

source /opt/ros/jazzy/setup.bash
sudo apt-get update
sudo apt-get install -y babeltrace ros-jazzy-ros2trace ros-jazzy-tracetools-analysis
ros2 run tracetools status
rosdep update
rosdep install -i -y --from-paths src --rosdistro jazzy --skip-keys test_tracetools