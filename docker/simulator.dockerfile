FROM ros:jazzy AS common-ws-builder

ARG DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN mkdir -p /faucon_ws/src
WORKDIR /faucon_ws
COPY src/common src
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    && rosdep install --from-paths src --ignore-src -r -y \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build \
    && rm -rf log/ build/ src/

FROM ros:jazzy AS gazebo-ws-builder

ARG DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN mkdir -p /faucon_ws/src
COPY --from=common-ws-builder /faucon_ws /faucon_ws
WORKDIR /faucon_ws
COPY src/gazebo src
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    && rosdep install --from-paths src --ignore-src -r -y \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && . /faucon_ws/install/setup.sh \
    && colcon build \
    && rm -rf log/ build/ src/


FROM ros:jazzy AS vehicle-ws-builder

ARG DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN mkdir -p /faucon_ws/src
COPY --from=common-ws-builder /faucon_ws /faucon_ws
WORKDIR /faucon_ws
COPY src/vehicle src
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    && rosdep install --from-paths src --ignore-src -r -y \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && . /faucon_ws/install/setup.sh \
    && colcon build \
    && rm -rf log/ build/ src/

FROM ros:jazzy AS loc-ws-builder

ARG DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN mkdir -p /faucon_ws/src
COPY --from=common-ws-builder /faucon_ws /faucon_ws
WORKDIR /faucon_ws
COPY src/loc src
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    && rosdep install --from-paths src --ignore-src -r -y \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && . /faucon_ws/install/setup.sh \
    && colcon build \
    && rm -rf log/ build/ src/

FROM ros:jazzy AS nav-ws-builder

ARG DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN mkdir -p /faucon_ws/src
COPY --from=common-ws-builder /faucon_ws /faucon_ws
WORKDIR /faucon_ws
COPY src/nav src
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    && rosdep install --from-paths src --ignore-src -r -y \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && . /faucon_ws/install/setup.sh \
    && colcon build \
    && rm -rf log/ build/ src/

FROM ros:jazzy-ros-core

ARG DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-jazzy-ros2bag \
    ros-jazzy-rosbag2-storage-default-plugins \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rviz2 \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /faucon_ws/src
WORKDIR /faucon_ws
COPY src/dependencies src/dependencies

RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    python3-rosdep \
    && rosdep init \
    && rosdep update --rosdistro $ROS_DISTRO \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

COPY --from=common-ws-builder /faucon_ws /faucon_ws
COPY --from=gazebo-ws-builder /faucon_ws /faucon_ws
COPY --from=vehicle-ws-builder /faucon_ws /faucon_ws
COPY --from=loc-ws-builder /faucon_ws /faucon_ws
COPY --from=nav-ws-builder /faucon_ws /faucon_ws
COPY --from=grpc-ws-builder /faucon_ws /faucon_ws
COPY --from=logger-ws-builder /faucon_ws /faucon_ws

WORKDIR /

RUN echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo ". /faucon_ws/install/setup.bash" >> ~/.bashrc

# setup entrypoint
COPY docker/ros_entrypoint.sh /
ENTRYPOINT ["/bin/sh", "/ros_entrypoint.sh"]
CMD ["/bin/bash"]