FROM ros:jazzy

# install required packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers ros-jazzy-ros-gz \
    ros-jazzy-ackermann-msgs  \
    ros-jazzy-nav2-bringup \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-slam-toolbox \
    ros-jazzy-controller-manager && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /sim_ws/src

CMD ["bash"]