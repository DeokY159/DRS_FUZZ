#!/usr/bin/env bash
set -e

sudo apt-get update && sudo apt-get install -y ros-${ROS_DISTRO}-gazebo-* \
    ros-${ROS_DISTRO}-cartographer ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup \
  && rm -rf /var/lib/apt/lists/*

mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3.git

# for headless mode
origin_path="/root/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/empty_world.launch.py"
headless_path="/root/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/empty_world.headless.launch.py"
cp "$origin_path" "$headless_path"
sed -i '/ld\.add_action(gzclient_cmd)/s/^/# /' "$headless_path"

source /opt/ros/${ROS_DISTRO}/setup.sh
cd ~/turtlebot3_ws
if [ "$ASAN_ENABLED" = "true" ]; then
  # Debug + ASAN
  colcon build --parallel-worker 1 \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_C_FLAGS=-fsanitize=address \
      -DCMAKE_CXX_FLAGS=-fsanitize=address
else
  # Release
  colcon build --parallel-worker 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

echo "export TURTLEBOT3_MODEL=burger"             | sudo tee -a /etc/bash.bashrc
echo "source ~/turtlebot3_ws/install/setup.bash"  | sudo tee -a /etc/bash.bashrc
echo "source /usr/share/gazebo/setup.sh"          | sudo tee -a /etc/bash.bashrc
echo "source /usr/share/gazebo-11/setup.sh"       | sudo tee -a /etc/bash.bashrc


