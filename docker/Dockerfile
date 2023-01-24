FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-cv-bridge \
    ros-noetic-ros-numpy \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install numpy==1.23.5 matplotlib opencv-python

COPY . /opt/ros/noetic/share/lidar_to_cam

ENTRYPOINT [ "/ros_entrypoint.sh", "roslaunch", "/opt/ros/noetic/share/lidar_to_cam/launch/husky_lidar_projection.launch" ]
