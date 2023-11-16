ARG BASE_IMAGE=ghcr.io/open-rmf/rmf/rmf_demos
FROM $BASE_IMAGE

# Hammer overlay
RUN mkdir $HOME/hammer_overlay_ws
WORKDIR $HOME/hammer_overlay_ws
RUN mkdir src
RUN git clone https://github.com/open-rmf/rmf_internal_msgs -b luca/beacon_state src/rmf_internal_msgs
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select rmf_fleet_msgs
RUN rm -rf build devel src

RUN curl -sL https://deb.nodesource.com/setup_16.x | bash - && \
  apt-get update && apt-get install -y \
    nodejs \
    jq && \
  pip3 install pipenv && \
  npm config set unsafe-perm
