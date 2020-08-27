#-----------------
# Stage 1 - build
#-----------------

FROM ros:eloquent AS builder

# helpful packages for development
RUN apt-get update && apt-get install -y curl wget ssh bash-completion

# enable bash completion for root user
RUN sed -i '/bash_completion/,+2s/#\(.*\)/\1/' /root/.bashrc

# add gazebo/ignition repo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
  && wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
  && apt-get update

WORKDIR /root/rmf

RUN mkdir src && cd src && \
  git clone --depth 1 https://github.com/osrf/rmf_core && \
  git clone --depth 1 https://github.com/osrf/traffic_editor && \
  git clone --depth 1 https://github.com/osrf/rmf_schedule_visualizer && \
  # TODO: use master branch when this is merged https://github.com/osrf/soss/pull/42
  git clone --depth 1 -b feat/pubkey https://github.com/osrf/soss && \
  git clone --depth 1 https://github.com/osrf/rmf_demos && \
  git clone --depth 1 https://github.com/osrf/rmf-soss-ros2

# install ros dependencies
RUN rosdep update && rosdep install --from-paths src --ignore-src -yr

# other dependencies
# TODO: ros-eloquent-test-msgs could be included in rosdep, remove it when its fixed in soss.
RUN apt-get update && apt-get install -y g++-8 libignition-common3-dev libignition-plugin-dev ros-eloquent-test-msgs

# build rmf
ENV CXX=g++-8
RUN /ros_entrypoint.sh colcon build --merge-install

#----------
# Stage 2
#----------

FROM builder

# TODO: rosdep doesn't support installing only exec dependencies (https://github.com/ros-infrastructure/rosdep/pull/727)
#   When the PR is merged, we can do a multi-stage build and include only whats needed at runtime.
# FROM ros:eloquent
# COPY --from=0 /root/rmf/install /opt/rmf
RUN ln -s /root/rmf/install /opt/rmf

# TODO: these packages are missing in https://github.com/osrf/rmf_schedule_visualizer, remove these once its been fixed.
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-launch-xml && rm -rf /var/lib/apt/lists/*

ADD rmf_entrypoint.bash /
ENTRYPOINT ["/rmf_entrypoint.bash"]
CMD ["bash"]
