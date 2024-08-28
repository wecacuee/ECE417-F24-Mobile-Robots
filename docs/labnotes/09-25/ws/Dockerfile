FROM --patform=$BUILDPLATFORM dustynv/ros:foxy-ros-base-l4t-r32.5.0 AS base

RUN mkdir -p /root/wrk/ece417/ws
WORKDIR /root/wrk/ece417/ws
COPY . .
RUN apt-get update && apt-get install -y \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*
RUN source $ROS_ROOT/install/setup.bash && colcon build --symlink-install
