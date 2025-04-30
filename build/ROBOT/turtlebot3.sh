#!/usr/bin/env bash
set -e

apt-get update && apt-get install -y \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3 \
  && rm -rf /var/lib/apt/lists/*

mkdir -p /turtlebot3_ws/src
cd /turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git

export CXXFLAGS="-fsanitize=address,fuzzer-no-link -fno-omit-frame-pointer -O1"
export CFLAGS="-fsanitize=address,fuzzer-no-link -fno-omit-frame-pointer -O1"
export LDFLAGS="-fsanitize=address,fuzzer-no-link"

source /opt/ros/humble/setup.sh
rosdep update
cd /turtlebot3_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y

cd /turtlebot3_ws
colcon build --symlink-install \
  --cmake-args -DCMAKE_C_COMPILER=clang-16 -DCMAKE_CXX_COMPILER=clang++-16

echo 'export TURTLEBOT3_MODEL=burger' >> /etc/profile.d/turtlebot3.sh
