ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=docker.io/ros:$ROS_DISTRO-ros-core
FROM $BASE_IMAGE

RUN apt update && apt install -y curl ros-$ROS_DISTRO-rmf-*-msgs python3-venv

RUN curl -fsSL https://get.pnpm.io/install.sh | bash -
# shell runs in non-interactive mode, which does not source .bashrc so we need to set the PATH manually
ENV PNPM_HOME /root/.local/share/pnpm
ENV PATH "$PNPM_HOME:$PATH"

RUN pnpm env use --global lts

RUN rm -rf /var/lib/apt/lists
