from rmf_lift_msgs.msg import LiftState as RmfLiftState
from tortoise.models import Model

from .json_mixin import json_mixin


class LiftState(Model, json_mixin(RmfLiftState, lambda x: x.lift_name)):
    pass
