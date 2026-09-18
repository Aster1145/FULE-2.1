FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

# Install deps
RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-humble-ros-gzharmonic \
    ros-humble-ros-gzharmonic-bridge \
    ros-humble-ros-gzharmonic-image \
    ros-humble-slam-toolbox \
    ros-humble-px4-msgs \
    libceres-dev libeigen3-dev libopencv-dev \
    python3-pip git wget curl \
    gz-harmonic

RUN pip3 install ultralytics torch torchvision opencv-python numpy scipy scikit-learn transforms3d pyyaml

# PX4
RUN git clone https://github.com/PX4/PX4-Autopilot.git /root/PX4-Autopilot --recursive && \
    cd /root/PX4-Autopilot && bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

# Micro DDS Agent
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /root/Micro-XRCE-DDS-Agent && \
    cd /root/Micro-XRCE-DDS-Agent && mkdir build && cd build && cmake .. && make -j4 && make install && ldconfig

WORKDIR /root/autonomous_maze_drone
COPY . .

RUN rosdep init || true && rosdep update && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y || true"

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

CMD ["/bin/bash"]
