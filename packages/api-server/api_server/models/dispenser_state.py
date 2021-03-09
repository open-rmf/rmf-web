from rmf_dispenser_msgs.msg import DispenserState as RmfDispenerState
from tortoise.models import Model

from .json_mixin import json_mixin


class DispenserState(Model, json_mixin(RmfDispenerState, lambda x: x.guid)):
    pass
