# FROM nvidia/cudagl:11.0-base
FROM nvidia/opengl:base-ubuntu20.04

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get -qq update && apt-get -q -y install tzdata \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean

# ENV DEBIAN_FRONTEND noninteractive

RUN apt-get -qq update && apt-get -q -y install \
    gnupg2 \
    curl \
    ca-certificates \
    build-essential \
    git \
    tmux \
    nano \
    wget \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean

# setup sources.list and keys
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# install ROS (including dependencies for building packages) and other packages
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get -qq update

# Install blender
RUN wget https://mirrors.ocf.berkeley.edu/blender/release/Blender3.3/blender-3.3.1-linux-x64.tar.xz 
RUN tar xvf blender-3.3.1-linux-x64.tar.xz  -C /usr/local/
RUN rm blender-3.3.1-linux-x64.tar.xz

WORKDIR /usr/local/blender-3.3.1-linux-x64/3.3/python/bin
RUN wget https://bootstrap.pypa.io/get-pip.py
RUN ./python3.10 get-pip.py

# Blender python deps
RUN ./python3.10 -m pip install numpy matplotlib

WORKDIR /

RUN apt-get -q -y install \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-noetic-desktop \
    && rosdep init \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean



# Change default python version to 3.9
# RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.9 2 

# sdformat8-sdf conflicts with sdformat-sdf installed from gazebo
# so we need to workaround this using a force overwrite
# Do this before installing ign-gazebo
# (then install ign-blueprint and ros to ign bridge)
RUN curl -sSL http://get.gazebosim.org | sh

RUN apt-get -qq update
RUN apt-get -q -y install ignition-fortress  --fix-missing
RUN apt-get -q -y install ros-noetic-ros-ign  --fix-missing
RUN rm -rf /var/lib/apt/lists/*
RUN apt-get -qq clean

RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python3 get-pip.py

RUN pip install cookiecutter rospkg numpy

COPY . /catkin_ws/src/fields_ignition
WORKDIR /catkin_ws

RUN apt-get update

RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y  || echo "There were some errors during rosdep install"
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make

WORKDIR /catkin_ws/src/fields_ignition

RUN source /catkin_ws/devel/setup.sh
RUN export PATH=/usr/local/blender-3.3.1-linux-x64:$PATH

RUN pip install -r api/requirements.txt

# CMD source devel/setup.bash && roslaunch fields_ignition field.launch world_dir:=${WORLD_DIR}