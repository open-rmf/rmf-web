ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=docker.io/ros:$ROS_DISTRO-ros-base
FROM $BASE_IMAGE
ARG BRANCH=main

### build minimal rmf

RUN apt update && apt install -y curl

# # fetch sources
RUN mkdir -p /rmf && cd /rmf \
  && curl -sL https://github.com/open-rmf/rmf_internal_msgs/archive/refs/heads/$BRANCH.tar.gz -o rmf_internal_msgs.tar.gz \
  && curl -sL https://github.com/open-rmf/rmf_building_map_msgs/archive/refs/heads/$BRANCH.tar.gz -o rmf_building_map_msgs.tar.gz \
  && mkdir -p /rmf/src/rmf/rmf_internal_msgs && tar zxf rmf_internal_msgs.tar.gz -C /rmf/src/rmf/rmf_internal_msgs --strip-components=1 && rm rmf_internal_msgs.tar.gz \
  && mkdir -p /rmf/src/rmf/rmf_building_map_msgs && tar zxf rmf_building_map_msgs.tar.gz -C /rmf/src/rmf/rmf_building_map_msgs --strip-components=1 && rm rmf_building_map_msgs.tar.gz

RUN rosdep update && rosdep install --from-paths /rmf/src -yi

RUN cd /rmf \
  && . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --merge-install --install-base /opt/rmf --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf /rmf

# install tools for rmf-web

RUN curl -fsSL https://get.pnpm.io/install.sh | bash -
# shell runs in non-interactive mode, which does not source .bashrc so we need to set the PATH manually
ENV PNPM_HOME /root/.local/share/pnpm
ENV PATH "$PNPM_HOME:$PATH"

RUN pnpm env use --global lts

RUN apt update && apt install -y python3-venv
