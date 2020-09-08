#-----------------
# Stage 1 - build
#-----------------

FROM ros:eloquent AS builder

# add gazebo/ignition repo
RUN apt-get update && apt-get install -y curl wget \
  && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
  && wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
  && apt-get update

WORKDIR /root/rmf

RUN mkdir src && cd src && \
  git clone --depth 1 https://github.com/osrf/rmf_core && \
  git clone --depth 1 https://github.com/osrf/traffic_editor && \
  git clone --depth 1 https://github.com/osrf/rmf_schedule_visualizer && \
  git clone --depth 1 https://github.com/osrf/soss && \
  git clone --depth 1 https://github.com/osrf/rmf_demos && \
  git clone --depth 1 https://github.com/osrf/rmf-soss-ros2

# install ros dependencies
RUN rosdep update && rosdep install --from-paths src --ignore-src -yr

# other dependencies
RUN apt-get update && apt-get install -y g++-8 \
  # needed by traffic editor
  libignition-common3-dev libignition-plugin-dev \
  # needed by soss
  ros-eloquent-test-msgs \
  && rm -rf /var/lib/apt/lists/*

# build rmf
ENV CXX=g++-8
RUN /ros_entrypoint.sh colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

#----------
# Stage 2
#----------

FROM builder

WORKDIR /root

# TODO: rosdep doesn't support installing only exec dependencies (https://github.com/ros-infrastructure/rosdep/pull/727)
#   When the PR is merged, we can do a multi-stage build and include only whats needed at runtime.
# FROM ros:eloquent
# COPY --from=0 /root/rmf/install /opt/rmf
# RUN rosdep ...
COPY --from=builder /root/rmf/install /opt/rmf
RUN rm -rf /root/rmf

# TODO: these packages are missing in https://github.com/osrf/rmf_schedule_visualizer, remove these once its been fixed.
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-launch-xml && rm -rf /var/lib/apt/lists/*

ADD rmf_entrypoint.bash /
ENTRYPOINT ["/rmf_entrypoint.bash"]
CMD ["bash"]
