FROM rmf/rmf

RUN curl -sL https://deb.nodesource.com/setup_12.x | bash - && \
  apt-get install -y nodejs

# TODO: need a new version of soss, remove this when the feature is merged and released
RUN cd /root/rmf/src && rm -rf soss && \
  git clone --depth 1 -b feat/pubkey https://github.com/osrf/soss && \
  cd /root/rmf && rm -rf build/soss-websocket install/soss-websocket && /entrypoint.bash colcon build

WORKDIR /root
