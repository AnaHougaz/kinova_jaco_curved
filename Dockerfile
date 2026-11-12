FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git \
    python3-catkin-tools \
    python3-rosdep \
    python3-pip \
    ros-noetic-moveit \
    ros-noetic-moveit-visual-tools \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-position-controllers \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/catkin_ws/src

WORKDIR /root/catkin_ws/src

RUN git clone https://github.com/Kinovarobotics/kinova-ros.git

WORKDIR /root/catkin_ws

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y"

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

CMD ["bash"]