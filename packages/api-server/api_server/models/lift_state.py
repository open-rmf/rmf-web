from rmf_lift_msgs.msg import LiftState as RmfLiftState

from .json_model import json_model


class LiftState(json_model(RmfLiftState, lambda x: x.lift_name)):
    pass
