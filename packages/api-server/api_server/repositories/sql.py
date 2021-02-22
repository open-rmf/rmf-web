import logging
from typing import Dict

from building_map_msgs.msg import Door as RmfDoor
from rmf_door_msgs.msg import DoorState as RmfDoorState

from ..models import Door, DoorState


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
        self.logger.debug(f"written door_state ({door_state.door_name}) to database")

    async def read_door_states(self) -> Dict[str, RmfDoorState]:
        all_states = await DoorState.all()
        return {x.door_name: x.to_rmf() for x in all_states}

    async def update_door(self, rmf_door: RmfDoor):
        door = Door.from_rmf(rmf_door)
        await Door.update_or_create(
            {
                "v1_x": door.v1_x,
                "v1_y": door.v1_y,
                "v2_x": door.v2_x,
                "v2_y": door.v2_y,
                "door_type": door.door_type,
                "motion_range": door.motion_range,
                "motion_direction": door.motion_direction,
            },
            name=door.name,
        )
        self.logger.debug(f"written door ({door.name}) to database")
