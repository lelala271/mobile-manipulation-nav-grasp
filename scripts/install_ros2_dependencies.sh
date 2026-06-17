#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-rviz2 \
  ros-humble-tf2-tools \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization

rosdep update
