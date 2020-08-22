FROM rmf/rmf

RUN cd /root/rmf/src && rm -rf soss && \
  # TODO: need a new version of soss, remove this when the feature is merged and released
  git clone --depth 1 -b feat/pubkey https://github.com/osrf/soss && \
  cd /root/rmf && rm -rf build/soss-websocket install/soss-websocket && /entrypoint.bash colcon build

ADD soss.yaml.in /root/romi-dashboard/
ADD certs /root/romi-dashboard/certs
RUN sed 's/{{pwd}}/\/root\/romi-dashboard/' /root/romi-dashboard/soss.yaml.in > /root/romi-dashboard/soss.yaml

ENTRYPOINT ["/entrypoint.bash"]
CMD ["soss", "/root/romi-dashboard/soss.yaml"]
