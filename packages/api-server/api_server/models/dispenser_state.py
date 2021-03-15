from rmf_dispenser_msgs.msg import DispenserState as RmfDispenserState

from .json_model import json_model


class DispenserState(json_model(RmfDispenserState, lambda x: x.guid)):
    pass
