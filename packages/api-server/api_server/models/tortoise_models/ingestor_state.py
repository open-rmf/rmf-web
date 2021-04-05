from rmf_ingestor_msgs.msg import IngestorState as RmfIngestorState

from .json_model import json_model


class IngestorState(json_model(RmfIngestorState, lambda x: x.guid)):
    pass
