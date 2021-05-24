from pydantic import BaseModel

from .health import BasicHealth
from .ros_pydantic import rmf_fleet_msgs

FleetState = rmf_fleet_msgs.FleetState
RobotState = rmf_fleet_msgs.RobotState
RobotMode = rmf_fleet_msgs.RobotMode
RobotHealth = BasicHealth
Location = rmf_fleet_msgs.Location


class Fleet(BaseModel):
    name: str
