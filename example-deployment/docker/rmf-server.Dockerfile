FROM rmf-web/rmf-server:build as stage0

FROM ros:foxy-ros-base-focal

SHELL ["bash", "-c"]

COPY rmf/rmf_internal_msgs/rmf_charger_msgs /root/rmf_ws/src/rmf_charger_msgs
COPY rmf/rmf_internal_msgs/rmf_dispenser_msgs /root/rmf_ws/src/rmf_dispenser_msgs
COPY rmf/rmf_internal_msgs/rmf_door_msgs /root/rmf_ws/src/rmf_door_msgs
COPY rmf/rmf_internal_msgs/rmf_fleet_msgs /root/rmf_ws/src/rmf_fleet_msgs
COPY rmf/rmf_internal_msgs/rmf_ingestor_msgs /root/rmf_ws/src/rmf_ingestor_msgs
COPY rmf/rmf_internal_msgs/rmf_lift_msgs /root/rmf_ws/src/rmf_lift_msgs
COPY rmf/rmf_internal_msgs/rmf_task_msgs /root/rmf_ws/src/rmf_task_msgs
COPY rmf/rmf_internal_msgs/rmf_traffic_msgs /root/rmf_ws/src/rmf_traffic_msgs
COPY rmf/rmf_internal_msgs/rmf_workcell_msgs /root/rmf_ws/src/rmf_workcell_msgs
COPY rmf/rmf_building_map_msgs /root/rmf_ws/src/rmf_building_map_msgs

RUN . /opt/ros/foxy/setup.bash && cd /root/rmf_ws && \
  colcon build --merge-install --install-base /opt/rmf --cmake-args -DCMAKE_BUILD_TYPE=Release && \
  rm -rf /root/rmf_ws

COPY --from=0 /root/rmf-web/packages/api-server/dist/ /root/rmf-server/
RUN apt-get update && apt-get install -y python3-pip
RUN cd /root/rmf-server && \
  pip3 install $(ls -1 | grep '.*.whl')[postgres] && \
  rm -rf /root/rmf-server

RUN echo -e '#!/bin/bash\n\
  . /opt/rmf/setup.bash\n\
  exec rmf_api_server "$@"\n\
  ' > /docker-entry-point.sh && chmod +x /docker-entry-point.sh
ENTRYPOINT ["/docker-entry-point.sh"]
