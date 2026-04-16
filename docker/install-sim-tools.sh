#!/usr/bin/env bash
# Installs Gazebo Harmonic, RViz, Nav2, SLAM Toolbox, and dev tooling.
# Shared between the `dev` and `dev-vision` Dockerfile stages.
set -euo pipefail

: "${ROS_DISTRO:?ROS_DISTRO must be set}"

apt-get update
apt-get install -y --no-install-recommends \
    ros-"$ROS_DISTRO"-ros-gz \
    ros-"$ROS_DISTRO"-rviz2 \
    ros-"$ROS_DISTRO"-gz-ros2-control \
    curl \
    lsb-release \
    gnupg \
    ca-certificates

curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

apt-get update
apt-get install -y --no-install-recommends \
    gz-harmonic \
    ros-"$ROS_DISTRO"-rqt-graph \
    ros-"$ROS_DISTRO"-teleop-twist-keyboard \
    ros-"$ROS_DISTRO"-teleop-twist-joy \
    ros-"$ROS_DISTRO"-navigation2 \
    ros-"$ROS_DISTRO"-nav2-bringup \
    ros-"$ROS_DISTRO"-slam-toolbox

apt-get clean
rm -rf /var/lib/apt/lists/*
