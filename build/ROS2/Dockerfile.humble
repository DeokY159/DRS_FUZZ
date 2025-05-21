ARG ROS_DISTRO=humble
FROM ubuntu:22.04

ARG TARGET_ROBOT

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y \
      curl gnupg2 lsb-release locales vim \
      software-properties-common \
      build-essential cmake git wget \
      python3-pip \
      libacl1-dev libtinyxml2-dev libssl-dev \
      && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository universe \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list \
 && apt-get update \
 && apt-get install -y ros-dev-tools python3-rosdep python3-colcon-common-extensions python3-vcstool \
      libasio-dev libxaw7-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libfreetype6-dev  libbullet-dev

RUN mkdir -p ~/ros2_humble/src \
 && cd ~/ros2_humble \
 && vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src \
 && rosdep init \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" -y

RUN cd ~/ros2_humble \
 && MAKEFLAGS="-j1 -l1" colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_FLAGS=-fsanitize=address -DCMAKE_CXX_FLAGS=-fsanitize=address
RUN rm -rf /var/lib/apt/lists/* && apt-get clean && apt-get update

# Copy and run the robot-specific installation script
COPY ${TARGET_ROBOT}.sh /root/${TARGET_ROBOT}.sh
RUN chmod +x /root/${TARGET_ROBOT}.sh \
 && bash -c "source ~/ros2_humble/install/setup.bash && /root/${TARGET_ROBOT}.sh"

ENV LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libasan.so.6 /usr/lib/x86_64-linux-gnu/libstdc++.so.6"
ENV ASAN_OPTIONS="detect_leaks=0:new_delete_type_mismatch=0:handle_segv=0:abort_on_error=0:symbolize=1"
RUN echo "source ~/ros2_humble/install/setup.bash" >> /etc/bash.bashrc

ENTRYPOINT ["/bin/bash"]
