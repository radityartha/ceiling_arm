# ====================================================
# ✅ Base: ROS 2 Humble on Ubuntu 22.04
# ====================================================
FROM osrf/ros:humble-desktop

# ----------------------------------------------------
# 🧩 Environment setup
# ----------------------------------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/ros2_ws

# ----------------------------------------------------
# 🧰 System and ROS essentials
# ----------------------------------------------------
RUN apt-get update && apt-get install -y \
    tree \
    software-properties-common \
    curl \
    gnupg \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    nano \
    wget \
    sudo \
    build-essential \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-python \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------
# 🧩 Python dependencies
# ----------------------------------------------------
RUN pip3 install --no-cache-dir wheel numpy pymodbus pyserial

# ----------------------------------------------------
# 🧩 Initialize rosdep
# ----------------------------------------------------
RUN [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init && \
    rosdep update || echo "rosdep already initialized"

# ----------------------------------------------------
# 📂 Create workspace
# ----------------------------------------------------
WORKDIR ${WORKSPACE}
RUN mkdir -p src

# ----------------------------------------------------
# 🧩 Copy project sources
# ----------------------------------------------------
COPY ros2_ws/src ${WORKSPACE}/src
COPY config ${WORKSPACE}/../config
COPY scripts ${WORKSPACE}/../scripts
COPY dependencies/kortex_api-2.6.0.post3-py3-none-any.whl ${WORKSPACE}/../dependencies/

# ----------------------------------------------------
# 🧩 Install Kinova Kortex API (if needed)
# ----------------------------------------------------
RUN pip3 install ${WORKSPACE}/../dependencies/kortex_api-2.6.0.post3-py3-none-any.whl || echo "Optional Kortex API"

# ----------------------------------------------------
# 🧩 Clone additional dependencies
# ----------------------------------------------------
WORKDIR ${WORKSPACE}/src
RUN git clone -b humble https://github.com/PickNikRobotics/ros2_robotiq_grippers.git dependencies/ros2_robotiq_grippers \
    || echo "Warning: robotiq_grippers clone failed"

RUN git clone -b humble https://github.com/PickNikRobotics/picknik_controllers.git dependencies/picknik_controllers \
    || echo "Warning: picknik_controllers clone failed"

# ----------------------------------------------------
# 🧩 Install ROS dependencies (skip missing)
# ----------------------------------------------------
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths ${WORKSPACE}/src --ignore-src -r -y \
    --skip-keys='picknik_reset_fault_controller picknik_twist_controller robotiq_description'" \
    || echo 'rosdep install completed with skipped keys'

# ----------------------------------------------------
# 🧩 Clean build artifacts before build
# ----------------------------------------------------
RUN rm -rf ${WORKSPACE}/build ${WORKSPACE}/install ${WORKSPACE}/log

# ----------------------------------------------------
# 🧩 Build the workspace
# ----------------------------------------------------
WORKDIR ${WORKSPACE}

# Remove any previous build/install/log folders to avoid duplicate targets
RUN rm -rf build install log

# Then build the workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
                  colcon build --symlink-install --cmake-clean-cache"

# ----------------------------------------------------
# 🧩 Source setup for interactive shells
# ----------------------------------------------------
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc && \
    echo 'source ${WORKSPACE}/install/setup.bash' >> ~/.bashrc

# ----------------------------------------------------
# 🧩 Default entrypoint
# ----------------------------------------------------
ENTRYPOINT ["/bin/bash", "-c"]
CMD ["bash"]
