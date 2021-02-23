import logging
from typing import Dict, Sequence

import tortoise
from building_map_msgs.msg import Door as RmfDoor
from rmf_door_msgs.msg import DoorState as RmfDoorState

from ..models import Door, DoorHealth, DoorState
from .rmf import RmfRepository


class SqlRepository(RmfRepository):
    """
    NOTE: tortoise-orm must already be inited before using any of the methods in this class.
    """

    def __init__(self, logger: logging.Logger = None):
        self.logger = logger or logging.getLogger(self.__class__.__name__)

    async def update_door_state(self, door_state: RmfDoorState):
        sql_door_state = DoorState.from_rmf(door_state)
        await DoorState.update_or_create(
            {
                "current_mode": sql_door_state.current_mode,
                "door_time": sql_door_state.door_time,
            },
            door_name=sql_door_state.door_name,
        )
        self.logger.debug(
            f"written door_state ({sql_door_state.door_name}) to database"
        )

    async def read_door_states(self) -> Dict[str, RmfDoorState]:
        all_states = await DoorState.all()
        return {x.door_name: x.to_rmf() for x in all_states}

    async def update_door(self, door: RmfDoor):
        sql_door = Door.from_rmf(door)
        await Door.update_or_create(
            {
                "v1_x": sql_door.v1_x,
                "v1_y": sql_door.v1_y,
                "v2_x": sql_door.v2_x,
                "v2_y": sql_door.v2_y,
                "door_type": sql_door.door_type,
                "motion_range": sql_door.motion_range,
                "motion_direction": sql_door.motion_direction,
            },
            name=door.name,
        )
        self.logger.debug(f"written door ({sql_door.name}) to database")

    async def read_door(self, name: str):
        door = await Door.filter(name=name).first()
        if door:
            return Door.to_rmf(door)
        return None

    async def sync_doors(self, doors: Sequence[RmfDoor]) -> None:
        async with tortoise.transactions.in_transaction():
            await Door.all().delete()
            for door in doors:
                await self.update_door(door)

    async def update_door_health(self, door_health: DoorHealth):
        await DoorHealth.update_or_create(vars(door_health))
        self.logger.debug(f'written door health for "{door_health.name}" to database')

    async def read_door_health(self, door_name: str):
        return await DoorHealth.filter(name=door_name).first()
