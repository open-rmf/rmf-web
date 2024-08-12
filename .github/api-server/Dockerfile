ARG BASE_IMAGE
FROM $BASE_IMAGE
ARG BRANCH=main
ARG ROS_DISTRO=jazzy

### Build api-server

RUN apt update && apt install -y python3-pip ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# fetch sources
RUN mkdir -p /ws \
  && curl -L https://github.com/open-rmf/rmf-web/archive/$BRANCH.tar.gz -o rmf_web.tar.gz \
  && tar zxf rmf_web.tar.gz -C /ws --strip-components=1

# install deps
RUN cd /ws \
  && pnpm install --filter api-server...

# cleanup
RUN rm -rf \
    /var/lib/apt/lists \
    /dist

# WORKDIR /ws/packages/api-server
# ENTRYPOINT ["bash", "-c", ". /opt/ros/$ROS_DISTRO/setup.bash && pnpm start"]
ENTRYPOINT ["bash"]