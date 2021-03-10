import logging
from typing import Dict, Optional, Sequence

import tortoise
from building_map_msgs.msg import Door as RmfDoor
from rmf_dispenser_msgs.msg import DispenserState as RmfDispenserState
from rmf_door_msgs.msg import DoorState as RmfDoorState
from rmf_fleet_msgs.msg import FleetState as RmfFleetState
from rmf_lift_msgs.msg import LiftState as RmfLiftState

from ..models import (
    DispenserHealth,
    DispenserState,
    Door,
    DoorHealth,
    DoorState,
    FleetState,
    LiftHealth,
    LiftState,
    RobotHealth,
    get_robot_id,
)
from .rmf import RmfRepository


class SqlRepository(RmfRepository):
    """
    NOTE: tortoise-orm must already be inited before using any of the methods in this class.
    """

    def __init__(self, logger: logging.Logger = None):
        self.logger = logger or logging.getLogger(self.__class__.__name__)

    async def _save(self, value: tortoise.Model, **filter_args):
        existing = await value.filter(**filter_args).first()
        if existing:
            await value.save(force_update=True)
        else:
            await value.save(force_create=True)

    async def update_door_state(self, door_state: RmfDoorState):
        sql_door_state = await DoorState.from_rmf(door_state)
        await self._save(sql_door_state, door_name=sql_door_state.door_name)
        self.logger.debug(
            f"written door_state ({sql_door_state.door_name}) to database"
        )

    async def read_door_states(self) -> Dict[str, RmfDoorState]:
        all_states = await DoorState.all()
        return {x.door_name: x.to_rmf() for x in all_states}

    async def update_door(self, door: RmfDoor):
        sql_door = await Door.from_rmf(door)
        await self._save(sql_door, name=sql_door.name)
        self.logger.debug("written door to database")

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
        await self._save(door_health, id_=door_health.id_)
        self.logger.debug("written door health for to database")

    async def read_door_health(self, door_name: str) -> Optional[DoorHealth]:
        return await DoorHealth.filter(id_=door_name).first()

    async def update_lift_state(self, lift_state: RmfLiftState):
        state = LiftState().update_from_rmf(lift_state)
        await self._save(state, id_=state.id_)
        self.logger.debug("written lift_state to database")

    async def read_lift_states(self) -> Dict[str, RmfLiftState]:
        all_states = await LiftState.all()
        return {x.id_: x.to_rmf() for x in all_states}

    async def update_lift_health(self, lift_health: LiftHealth) -> None:
        await self._save(lift_health, id_=lift_health.id_)
        self.logger.debug("written lift health for to database")

    async def read_lift_health(self, lift_name: str) -> Optional[LiftHealth]:
        return await LiftHealth.filter(id_=lift_name).first()

    async def update_dispenser_state(self, dispenser_state: RmfDispenserState):
        state = DispenserState().update_from_rmf(dispenser_state)
        await self._save(state, id_=state.id_)
        self.logger.debug("written dispenser_state to database")

    async def read_dispenser_states(self) -> Dict[str, RmfDispenserState]:
        all_states = await DispenserState.all()
        return {x.id_: x.to_rmf() for x in all_states}

    async def update_dispenser_health(self, dispenser_health: DispenserHealth) -> None:
        await self._save(dispenser_health, id_=dispenser_health.id_)
        self.logger.debug("written dispenser health for to database")

    async def read_dispenser_health(self, guid: str) -> Optional[DispenserHealth]:
        return await DispenserHealth.filter(id_=guid).first()

    async def update_fleet_state(self, fleet_state: RmfFleetState):
        state = FleetState().update_from_rmf(fleet_state)
        await self._save(state, id_=state.id_)
        self.logger.debug("written fleet_state to database")

    async def read_fleet_states(self) -> Dict[str, FleetState]:
        all_states = await FleetState.all()
        return {x.id_: x.to_rmf() for x in all_states}

    async def update_robot_health(self, robot_health: RobotHealth) -> None:
        await self._save(robot_health, id_=robot_health.id_)
        self.logger.debug("written robot health for to database")

    async def read_robot_health(
        self, fleet_name: str, robot_name: str
    ) -> Optional[RobotHealth]:
        return await RobotHealth.filter(
            id_=get_robot_id(fleet_name, robot_name)
        ).first()
