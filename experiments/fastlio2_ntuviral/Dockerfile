# FAST-LIO2 (brytsknguyen/FAST_LIO fork, ships the NTU VIRAL config) built for
# offline evaluation on eee_03.bag. See HANDOFF.md for why this exists.
#
# Repos and pinned refs (both track "master" at build time; re-pull to update):
#   https://github.com/Livox-SDK/Livox-SDK          - C SDK, build dep of the ROS driver
#   https://github.com/Livox-SDK/livox_ros_driver   - hard catkin dep of fast_lio,
#                                                      even though eee_03 is an Ouster
#   https://github.com/brytsknguyen/FAST_LIO         - NTU VIRAL authors' fork;
#                                                      config/ntuviral.yaml +
#                                                      launch/run_ntuviral.launch
#     (its include/ikd-Tree is a git submodule - must `git submodule update --init`)
FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# libpcl-dev / eigen_conversions / pcl-ros / tf2-ros ship with desktop-full, but
# installed explicitly so a lighter base image would still work.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    python3-dev python3-matplotlib \
    libeigen3-dev libpcl-dev \
    ros-noetic-pcl-ros ros-noetic-tf ros-noetic-tf2-ros \
    ros-noetic-eigen-conversions ros-noetic-rosbag ros-noetic-rviz \
    xvfb imagemagick x11-utils \
    && rm -rf /var/lib/apt/lists/*

# --- Livox-SDK: C library the ROS driver links against ---------------------
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK.git /opt/Livox-SDK && \
    mkdir -p /opt/Livox-SDK/build && cd /opt/Livox-SDK/build && \
    cmake .. && make -j"$(nproc)" && make install && ldconfig

# --- Catkin workspace --------------------------------------------------------
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git

RUN git clone --depth 1 https://github.com/brytsknguyen/FAST_LIO.git fast_lio && \
    cd fast_lio && git submodule update --init --recursive

WORKDIR $CATKIN_WS
RUN source /opt/ros/noetic/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro noetic || true && \
    catkin_make

# Our own wrapper: reuses fast_lio's run_ntuviral.launch as-is (no retuning),
# adds a rosbag recorder on /Odometry, and skips the (slow, disk-hungry)
# whole-run PCD dump we don't need. See eval_wrapper.launch for detail.
COPY eval_wrapper.launch $CATKIN_WS/eval_wrapper.launch
COPY capture_entrypoint.sh $CATKIN_WS/capture_entrypoint.sh
RUN chmod +x $CATKIN_WS/capture_entrypoint.sh

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source $CATKIN_WS/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source $CATKIN_WS/devel/setup.bash && exec \"$@\"", "--"]
CMD ["/bin/bash"]
