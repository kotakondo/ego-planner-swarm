FROM ros:humble

LABEL Maintainer="Kota Kondo <kkondo@mit.edu>" \
      Description="Ubuntu 22.04, ROS2 humble for DYNUS"

# Set environment variables
##########################################
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# Usuals
##########################################
RUN rm -rf /var/lib/apt/lists/*
RUN apt update
RUN apt upgrade -y
RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -q -y --no-install-recommends git tmux vim wget tmuxp make

# Making directories and setting up the environment
##########################################
WORKDIR /home
RUN mkdir code

# Install dependencies
##########################################

#TODO: now we use gazebo_ros_pkgs from source, so we don't need to install ros-${ROS_DISTRO}-gazebo-* and ros-${ROS_DISTRO}-gazebo-ros-pkgs i think - need to double check

RUN apt-get install -y  ros-${ROS_DISTRO}-octomap \
                        ros-${ROS_DISTRO}-octomap-msgs \
                        ros-${ROS_DISTRO}-octomap-ros \
                        ros-${ROS_DISTRO}-octomap-rviz-plugins \
                        ros-${ROS_DISTRO}-gazebo-* \
                        ros-${ROS_DISTRO}-pcl-conversions \
                        ros-${ROS_DISTRO}-example-interfaces \
                        ros-${ROS_DISTRO}-pcl-ros \
                        ros-${ROS_DISTRO}-rviz2 \
                        ros-${ROS_DISTRO}-rqt-gui \
                        ros-${ROS_DISTRO}-rqt-gui-py \
                        ros-${ROS_DISTRO}-tf2-tools \
                        ros-${ROS_DISTRO}-tf-transformations

RUN apt install -y  ros-${ROS_DISTRO}-desktop \
                        ros-dev-tools \
                        ros-${ROS_DISTRO}-turtlesim \
                        ros-${ROS_DISTRO}-rqt* \
                        ros-${ROS_DISTRO}-rviz2 \
                        ros-${ROS_DISTRO}-rviz-common \ 
                        libpcl-dev \
                        build-essential
                        # ros-${ROS_DISTRO}-gazebo-ros-pkgs \

# Install Gazebo for the suitable ROS version
# RUN curl -sSL http://get.gazebosim.org | sh

# Install Ego-Swarm and dependencies
##########################################
RUN mkdir -p /home/kota/code/ego_swarm_ws/src
WORKDIR /home/kota/code/dynus_ws/src
# dummy arg to invalidate cache and build from here
ARG DUMMY=unknown
RUN echo DUMMY=${DUMMY}
RUN git clone https://github.com/kotakondo/ego-planner-swarm.git
WORKDIR /home/code/dynus_ws/src/dynus
RUN git switch dev-non-uniform
WORKDIR /home/code/dynus_ws/src
RUN git clone https://kotakondo:${GIT_ACCESS_TOKEN}@github.com/kotakondo/dynus_interfaces.git
RUN git clone https://github.com/kotakondo/octomap_mapping.git
RUN git clone https://github.com/kotakondo/realsense_gazebo_plugin.git
RUN git clone https://github.com/kotakondo/livox_laser_simulation_ros2.git
RUN git clone https://github.com/kotakondo/octomap_rviz_plugins.git
RUN git clone https://github.com/kotakondo/gazebo_ros_pkgs.git
RUN mkdir -p /home/code/decomp_ws/src
WORKDIR /home/code/decomp_ws/src
RUN git clone https://github.com/kotakondo/DecompROS2.git
RUN mkdir -p /home/code/livox_ws/src
WORKDIR /home/code/livox_ws/src
RUN git clone https://github.com/kotakondo/livox_ros_driver2.git
WORKDIR /home/code
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git

# Gazebo
##########################################
### handle ALSA-related error
RUN echo    'pcm.!default { \
                  type plug \
                  slave.pcm "null" \
            }' >> /usr/share/alsa/alsa.conf

# For livox lidar
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib:/usr/local/include

# Alias
RUN echo 'alias tks="tmux kill-server"' >> ~/.bashrc
RUN echo 'alias sb="source ~/.bashrc"' >> ~/.bashrc
RUN echo 'alias eb="code ~/.bashrc"' >> ~/.bashrc
RUN echo 'alias gs="git status"' >> ~/.bashrc
RUN echo 'alias gp="git push"' >> ~/.bashrc
RUN echo 'alias roscd="cd ~/code/dynus_ws"' >> ~/.bashrc
RUN echo 'alias cb="roscd && colcon build && sb"' >> ~/.bashrc
RUN echo 'alias ss="roscd && source install/setup.bash"' >> ~/.bashrc
RUN echo 'alias cbd="clear && roscd && colcon build && ss"' >> ~/.bashrc
RUN echo 'alias cbm="clear && roscd && colcon build --packages-select ros2_mapper && ss"' >> ~/.bashrc
RUN echo 'alias cbsl="roscd && colcon build --symlink-install && sb"' >> ~/.bashrc
RUN echo 'alias cbps="roscd && colcon build --packages-select"' >> ~/.bashrc
RUN echo 'alias tf_visualize="ros2 run rqt_tf_tree rqt_tf_tree"' >> ~/.bashrc
RUN echo 'alias tks="tmux kill-server"' >> ~/.bashrc