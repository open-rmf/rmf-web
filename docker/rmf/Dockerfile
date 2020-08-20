FROM rmf/build

# TODO: rosdep doesn't support installing only exec dependencies (https://github.com/ros-infrastructure/rosdep/pull/727)
#   When the PR is merged, we can do a multi-stage build and include only whats needed at runtime.
# FROM ros:eloquent
# COPY --from=0 /root/rmf/install /opt/rmf
RUN ln -s /root/rmf/install /opt/rmf

# TODO: these packages are missing in https://github.com/osrf/rmf_schedule_visualizer, remove these once its been fixed.
RUN apt update && apt install -y ros-${ROS_DISTRO}-launch-xml

ADD entrypoint.bash /
ENTRYPOINT ["/entrypoint.bash"]
CMD ["bash"]
