ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=docker.io/ros:$ROS_DISTRO-ros-core
FROM $BASE_IMAGE

### install dependencies

RUN apt update && apt install -y curl ros-$ROS_DISTRO-rmf-*-msgs python3-venv

# these steps are required only for Jazzy as the Alert related messages are not released
RUN if [ "$ROS_DISTRO" = "jazzy" ]; then \
      apt install -y ros-dev-tools \
      && mkdir -p /rmf && cd /rmf \
      && curl -sL https://github.com/open-rmf/rmf_internal_msgs/archive/jazzy-alert-msgs.tar.gz -o jazzy-alert-msgs.tar.gz \
      && mkdir -p /rmf/src/alert_msgs && tar zxf jazzy-alert-msgs.tar.gz -C /rmf/src/alert_msgs --strip-components=1 && rm jazzy-alert-msgs.tar.gz \
      && cd /rmf \
      && . /opt/ros/$ROS_DISTRO/setup.sh \
      && colcon build --merge-install --install-base /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release \
      && rm -rf /rmf; \
    fi

# install tools for rmf-web

RUN curl -fsSL https://get.pnpm.io/install.sh | bash -
# shell runs in non-interactive mode, which does not source .bashrc so we need to set the PATH manually
ENV PNPM_HOME /root/.local/share/pnpm
ENV PATH "$PNPM_HOME:$PATH"

RUN pnpm env use --global lts

# cleanup
RUN rm -rf /var/lib/apt/lists
