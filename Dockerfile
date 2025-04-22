### Use ROS 2 Jazzy as the base image
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

### Use bash by default
SHELL ["/bin/bash", "-c"]

### Install basic tools
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
    curl \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

### Define working directory
ARG WS_DIR=/root/ws
ENV WS_DIR=${WS_DIR}
ENV WS_SRC_DIR=${WS_DIR}/src
ENV WS_INSTALL_DIR=${WS_DIR}/install
ENV WS_LOG_DIR=${WS_DIR}/log
WORKDIR ${WS_DIR}

### Add Gazebo package repository
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update

### Install Gazebo Harmonic
ARG GAZEBO_VERSION=harmonic
ENV GAZEBO_VERSION=${GAZEBO_VERSION}
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
    gz-${GAZEBO_VERSION} && \
    rm -rf /var/lib/apt/lists/*

### Install additional dependencies for ROS 2 and Gazebo
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-gz-ros2-control    \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-servo && \
    rm -rf /var/lib/apt/lists/*

### Copy over panda_gz_moveit2, then install dependencies and build
COPY ./ ${WS_SRC_DIR}/panda_gz_moveit2/
RUN rosdep update && \
    apt-get update && \
    rosdep install -y -r -i --rosdistro "${ROS_DISTRO}" --from-paths ${WS_SRC_DIR} && \
    rm -rf /var/lib/apt/lists/* && \
    source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release" && \
    rm -rf ${WS_LOG_DIR}

### Add workspace to the ROS entrypoint
### Source ROS workspace inside `~/.bashrc` to enable autocompletion
RUN sed -i '$i source "${WS_INSTALL_DIR}/local_setup.bash" --' /ros_entrypoint.sh && \
    sed -i '$a source "/opt/ros/${ROS_DISTRO}/setup.bash"' ~/.bashrc