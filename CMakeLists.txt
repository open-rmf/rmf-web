cmake_minimum_required(VERSION 3.5.0)

project(rmf_romi_dashboard)

set(rmf_msgs
  soss-rosidl
  soss-ros2
  rmf_door_msgs
  rmf_lift_msgs
  rmf_fleet_msgs
  rmf_traffic_msgs
  rmf_task_msgs
  rmf_dispenser_msgs
  building_map_msgs
)

foreach(msg ${rmf_msgs})
  find_package(${msg} REQUIRED)
endforeach(msg)

soss_rosidl_mix(
  PACKAGES std_msgs std_srvs geometry_msgs rmf_door_msgs rmf_lift_msgs rmf_fleet_msgs rmf_traffic_msgs rmf_task_msgs rmf_dispenser_msgs building_map_msgs
  MIDDLEWARES ros2
)