FROM ros:humble-ros-base-jammy

# Install standard C++ build tools, RViz2, MoveIt, and ROS 2 control
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    ros-humble-rviz2 \
    ros-humble-moveit \
    ros-humble-moveit-setup-assistant \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws