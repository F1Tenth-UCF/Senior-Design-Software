FROM ros:foxy

RUN apt-get update && apt-get install -y \
    python3-pip \
    && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Install wget for downloading Miniconda
RUN apt-get update && apt-get install -y \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-slam-toolbox \
    ros-foxy-tf-transformations \
    ros-foxy-cv-bridge \
    && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /sim_ws/src

RUN git clone --single-branch --branch AutoDRIVE-Devkit https://github.com/Tinker-Twins/AutoDRIVE.git
RUN mv "/AutoDRIVE/ADSS Toolkit/autodrive_ros2" "/sim_ws/src/"

RUN cd /sim_ws && colcon build

CMD ["bash"]