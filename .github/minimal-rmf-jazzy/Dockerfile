ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=docker.io/ros:$ROS_DISTRO-ros-core
FROM $BASE_IMAGE

# These branches are only required for Jazzy, as some required messages are not
# backported to be released.
ARG INTERNAL_MSGS_BRANCH=dd2951debfe53d0391896ecb9ddf07f027af05e9
ARG BUILDING_MAP_MSGS_BRANCH=e26cf73ec7b1f61bbd450a4f85450e0db20d6c72

### build minimal rmf

RUN apt update && apt install -y curl ros-dev-tools

# fetch sources
RUN mkdir -p /rmf && cd /rmf \
  && curl -sL https://github.com/open-rmf/rmf_internal_msgs/archive/$INTERNAL_MSGS_BRANCH.tar.gz -o rmf_internal_msgs.tar.gz \
  && curl -sL https://github.com/open-rmf/rmf_building_map_msgs/archive/$BUILDING_MAP_MSGS_BRANCH.tar.gz -o rmf_building_map_msgs.tar.gz \
  && mkdir -p /rmf/src/rmf/rmf_internal_msgs && tar zxf rmf_internal_msgs.tar.gz -C /rmf/src/rmf/rmf_internal_msgs --strip-components=1 && rm rmf_internal_msgs.tar.gz \
  && mkdir -p /rmf/src/rmf/rmf_building_map_msgs && tar zxf rmf_building_map_msgs.tar.gz -C /rmf/src/rmf/rmf_building_map_msgs --strip-components=1 && rm rmf_building_map_msgs.tar.gz

RUN cd /rmf \
  && . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --merge-install --install-base /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf /rmf

# install tools for rmf-web

RUN curl -fsSL https://get.pnpm.io/install.sh | bash -
# shell runs in non-interactive mode, which does not source .bashrc so we need to set the PATH manually
ENV PNPM_HOME /root/.local/share/pnpm
ENV PATH "$PNPM_HOME:$PATH"

RUN pnpm env use --global lts

RUN apt update && apt install -y python3-venv

# cleanup
RUN rm -rf /var/lib/apt/lists
