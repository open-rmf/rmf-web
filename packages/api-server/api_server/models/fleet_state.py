from rmf_fleet_msgs.msg import FleetState as RmfFleetState

from .json_model import json_model


class FleetState(json_model(RmfFleetState, lambda x: x.name)):
    pass
