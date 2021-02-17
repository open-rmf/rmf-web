import json
import logging
from typing import Dict

from rmf_door_msgs.msg import DoorState as RmfDoorState
from rosidl_runtime_py.convert import message_to_ordereddict

from ..models import DoorState


class SqlRepository:
    """
    NOTE: tortoise-orm must already be inited before using any of the methods in this class.
    """

    def __init__(self, logger: logging.Logger = None):
        self.logger = logger or logging.getLogger(self.__class__.__name__)

    async def update_door_state(self, rmf_door_state: RmfDoorState):
        door_state = DoorState.from_rmf(rmf_door_state)
        await DoorState.update_or_create(
            {
                "current_mode": door_state.current_mode,
                "door_time": door_state.door_time,
            },
            door_name=door_state.door_name,
        )
        self.logger.info(json.dumps(message_to_ordereddict(rmf_door_state)))

    async def read_door_states(self) -> Dict[str, RmfDoorState]:
        all_states = await DoorState.all()
        return {x.door_name: x.to_rmf() for x in all_states}
