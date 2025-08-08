ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE

FROM $BASE_IMAGE as base

# fetch sources
RUN mkdir -p /ws \
  && BRANCH_NAME=$ROS_DISTRO && if [ "$ROS_DISTRO" = "rolling" ]; then BRANCH_NAME=main; fi \
  && curl -L https://github.com/open-rmf/rmf-web/archive/$BRANCH_NAME.tar.gz -o rmf_web.tar.gz \
  && tar zxf rmf_web.tar.gz -C /ws --strip-components=1

# install deps
RUN cd /ws \
  && pnpm install --filter api-server...

# prepack
RUN cd /ws/packages/api-server \
  && pnpm run prepack

# cleanup
RUN rm -rf /var/lib/apt/lists

### Set up minimal image
FROM docker.io/library/ros:$ROS_DISTRO-ros-core

ARG ROS_DISTRO

RUN apt update && apt install -y python3-pip ros-$ROS_DISTRO-rmf-*-msgs ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# Copy over packed package and config file
RUN mkdir /ws
COPY --from=base /ws/packages/api-server/dist /ws
COPY --from=base /ws/packages/api-server/sqlite_local_config.py /ws/sqlite_local_config.py

# install
RUN pip3 install --break-system-packages --no-cache-dir /ws/api_server-*.whl \
  && rm /ws/api_server-*.whl

# cleanup
RUN rm -rf /var/lib/apt/lists

ENV RMF_API_SERVER_CONFIG=/ws/sqlite_local_config.py
WORKDIR /ws
ENTRYPOINT ["bash", "-c", ". /opt/ros/$ROS_DISTRO/setup.bash && mkdir -p run/cache && python3 -m api_server"]
