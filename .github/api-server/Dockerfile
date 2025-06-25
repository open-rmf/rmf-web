ARG BRANCH=main
ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE

FROM $BASE_IMAGE as base

ARG BRANCH

# fetch sources
RUN mkdir -p /ws \
  && curl -L https://github.com/open-rmf/rmf-web/archive/$BRANCH.tar.gz -o rmf_web.tar.gz \
  && tar zxf rmf_web.tar.gz -C /ws --strip-components=1

# install deps
RUN cd /ws \
  && pnpm install --filter api-server...

# cleanup
RUN rm -rf /var/lib/apt/lists

### Set up minimal image
FROM docker.io/library/ros:$ROS_DISTRO-ros-core

ARG ROS_DISTRO

RUN apt update && apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# according to rosdep, the messages have no deps so we can just copy the files directly in!
# this reduces the image size by ~50%.
COPY --from=base /opt/ros/$ROS_DISTRO/include /opt/ros/$ROS_DISTRO/include
COPY --from=base /opt/ros/$ROS_DISTRO/lib /opt/ros/$ROS_DISTRO/lib
COPY --from=base /opt/ros/$ROS_DISTRO/share /opt/ros/$ROS_DISTRO/share

# Copy over ws alongside the python virtual environment
RUN mkdir /ws
COPY --from=base /ws /ws

# cleanup
RUN rm -rf /var/lib/apt/lists

ENV RMF_API_SERVER_CONFIG /ws/packages/api-server/sqlite_local_config.py
WORKDIR /ws/packages/api-server
ENTRYPOINT ["bash", "-c", ". /opt/ros/$ROS_DISTRO/setup.bash && mkdir -p run/cache && ../../.venv/bin/pipenv run python -m api_server"]
