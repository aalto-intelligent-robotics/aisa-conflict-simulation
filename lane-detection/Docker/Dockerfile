# syntax=docker/dockerfile:1

FROM osrf/ros:noetic-desktop-full

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

RUN apt-get update 

RUN apt-get install -y git
RUN apt-get install -y tmux 
RUN apt-get install ros-noetic-cv-bridge
RUN apt-get install -y python3.8
RUN apt-get install -y python3-pip
RUN ln -s /usr/bin/python3.8 /usr/bin/python
RUN pip install --upgrade pip
RUN pip install psutil

# Install lane detection requirements
RUN pip install numpy
RUN pip install opencv-python
RUN pip install rospkg
# RUN pip install rospy
# RUN pip install rosgraph
RUN pip install torch
RUN pip install fastseg

CMD ["bash"]