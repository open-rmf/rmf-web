# Docker container with required packages to build rmf

FROM ros:eloquent

# helpful packages for development
RUN apt update && apt install -y curl wget ssh bash-completion

# enable bash completion for root user
RUN sed -i '/bash_completion/,+2s/#\(.*\)/\1/' /root/.bashrc

# add gazebo/ignition repo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
  && wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
  && apt update

WORKDIR /root/rmf

RUN mkdir src && cd src && \
  git clone --depth 1 -b 1.0.2 https://github.com/osrf/rmf_core && \
  git clone --depth 1 -b 1.0.0 https://github.com/osrf/traffic_editor && \
  git clone --depth 1 -b 1.0.0 https://github.com/osrf/rmf_schedule_visualizer && \
  git clone --depth 1 -b 2.0.0 https://github.com/osrf/soss && \
  git clone --depth 1 -b 1.0.0 https://github.com/osrf/rmf_demos && \
  git clone --depth 1 https://github.com/osrf/rmf-soss-ros2

# install ros dependencies
RUN rosdep update && rosdep install --from-paths src --ignore-src -yr

# other dependencies
# TODO: ros-eloquent-test-msgs could be included in rosdep, remove it when its fixed in soss.
RUN apt install -y g++-8 libignition-common3-dev libignition-plugin-dev ros-eloquent-test-msgs

# build rmf
ENV CXX=g++-8
RUN /ros_entrypoint.sh colcon build
