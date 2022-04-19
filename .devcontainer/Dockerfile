# See here for image contents: https://github.com/microsoft/vscode-dev-containers/tree/v0.195.0/containers/cpp/.devcontainer/base.Dockerfile
# [Choice] Debian / Ubuntu version (use Debian 11/9, Ubuntu 18.04/21.04 on local arm64/Apple Silicon): debian-11, debian-10, debian-9, ubuntu-21.04, ubuntu-20.04, ubuntu-18.04
#ARG VARIANT=ubuntu-20.04
#FROM mcr.microsoft.com/vscode/devcontainers/cpp:0-${VARIANT}

FROM osrf/ros:galactic-desktop

# [Optional] Uncomment this section to install additional packages.
 RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
     && apt-get -y install --no-install-recommends cmake build-essential python3-pip git\
     && pip install conan

RUN mkdir /workspace && cd /workspace && git clone https://github.com/Kinovarobotics/kortex.git

RUN conan config set general.revisions_enabled=1 && conan profile new default --detect 

RUN cd /workspace/kortex/api_cpp/examples/ && mkdir build && cd build && cmake .. && make 

# install packages
#RUN apt-get update && apt-get install -q -y --no-install-recommends \
#    dirmngr \
#    gnupg2 \
#    && rm -rf /var/lib/apt/lists/*

# setup sources.list
#RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list

# setup keys
#RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
#ENV LANG C.UTF-8
#ENV LC_ALL C.UTF-8

#ENV ROS_DISTRO foxy

# install ros2 packages
#RUN apt-get update && apt-get install -y --no-install-recommends \
#    ros-galactic-ros-core=0.9.3-2* \
#    && rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
#RUN rosdep init && \
#  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
#RUN colcon mixin add default \
#      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
#    colcon mixin update && \
#    colcon metadata add default \
#      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
#    colcon metadata update

# install ros2 packages
#RUN apt-get update && apt-get install -y --no-install-recommends \
#    ros-galactic-ros-base=0.9.3-2* \
 #   && rm -rf /var/lib/apt/lists/*