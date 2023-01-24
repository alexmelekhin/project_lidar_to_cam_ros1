#!/bin/bash

docker run -it --rm \
    --privileged \
    --net host \
    x86_64noetic/lidar_projection_node:latest
