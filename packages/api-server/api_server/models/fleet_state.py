from rmf_fleet_msgs.msg import FleetState as RmfFleetState
from tortoise.models import Model

from .json_mixin import json_mixin


class FleetState(Model, json_mixin(RmfFleetState, lambda x: x.name)):
    pass
