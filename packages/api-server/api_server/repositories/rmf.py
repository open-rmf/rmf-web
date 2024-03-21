from fastapi import Depends, HTTPException

from api_server.authenticator import user_dep
from api_server.logger import logger
from api_server.models import (
    BuildingMap,
    Dispenser,
    DispenserHealth,
    DispenserState,
    Door,
    DoorHealth,
    DoorState,
    Ingestor,
    IngestorHealth,
    IngestorState,
    Lift,
    LiftHealth,
    LiftState,
    Pagination,
    User,
)
from api_server.models import tortoise_models as ttm
from api_server.query import add_pagination


class RmfRepository:
    def __init__(self, user: User):
        self.user = user

    @staticmethod
    def _build_filter_params(**queries: dict):
        filter_params = {}
        for k, v in queries.items():
            if v is not None:
                filter_params[k] = v
        return filter_params

    async def get_bulding_map(self) -> BuildingMap | None:
        building_map = await ttm.BuildingMap.first()
        if building_map is None:
            return None
        if not isinstance(building_map.data, dict):
            logger.error(f"request is not a dict: {type(building_map.data)}")
            raise HTTPException(500)
        return BuildingMap(**building_map.data)

    async def get_doors(self) -> list[Door]:
        building_map = await self.get_bulding_map()
        if building_map is None:
            return []
        return [door for level in building_map.levels for door in level.doors]

    async def get_door_state(self, door_name: str) -> DoorState | None:
        door_state = await ttm.DoorState.get_or_none(id_=door_name)
        if door_state is None:
            return None
        return DoorState(**door_state.data)

    async def get_door_health(self, door_name: str) -> DoorHealth | None:
        door_health = await ttm.DoorHealth.get_or_none(id_=door_name)
        if door_health is None:
            return None
        return await DoorHealth.from_tortoise_orm(door_health)

    async def get_lifts(self) -> list[Lift]:
        building_map = await self.get_bulding_map()
        if building_map is None:
            return []
        return building_map.lifts

    async def get_lift_state(self, lift_name: str) -> LiftState | None:
        lift_state = await ttm.LiftState.get_or_none(id_=lift_name)
        if lift_state is None:
            return None
        return LiftState(**lift_state.data)

    async def get_lift_health(self, lift_name: str) -> LiftHealth | None:
        lift_health = await ttm.LiftHealth.get_or_none(id_=lift_name)
        if lift_health is None:
            return None
        return await LiftHealth.from_tortoise_orm(lift_health)

    async def get_dispensers(self) -> list[Dispenser]:
        states = await ttm.DispenserState.all()
        return [Dispenser(guid=state.data["guid"]) for state in states]

    async def get_dispenser_state(self, guid: str) -> DispenserState | None:
        dispenser_state = await ttm.DispenserState.get_or_none(id_=guid)
        if dispenser_state is None:
            return None
        return DispenserState(**dispenser_state.data)

    async def get_dispenser_health(self, guid: str) -> DispenserHealth | None:
        dispenser_health = await ttm.DispenserHealth.get_or_none(id_=guid)
        if dispenser_health is None:
            return None
        return await DispenserHealth.from_tortoise_orm(dispenser_health)

    async def get_ingestors(self) -> list[Ingestor]:
        states = await ttm.IngestorState.all()
        return [Ingestor(guid=state.data["guid"]) for state in states]

    async def get_ingestor_state(self, guid: str) -> IngestorState | None:
        ingestor_state = await ttm.IngestorState.get_or_none(id_=guid)
        if ingestor_state is None:
            return None
        return IngestorState(**ingestor_state.data)

    async def get_ingestor_health(self, guid: str) -> IngestorHealth | None:
        ingestor_health = await ttm.IngestorHealth.get_or_none(id_=guid)
        if ingestor_health is None:
            return None
        return await IngestorHealth.from_tortoise_orm(ingestor_health)

    async def query_users(
        self,
        pagination: Pagination,
        *,
        username: str | None = None,
        is_admin: bool | None = None,
    ) -> tuple[str]:
        filter_params = {}
        if username is not None:
            filter_params["username__istartswith"] = username
        if is_admin is not None:
            filter_params["is_admin"] = is_admin
        return (
            await add_pagination(
                ttm.User.filter(**filter_params), pagination
            ).values_list("username")
        )[0]


def rmf_repo_dep(user: User = Depends(user_dep)):
    return RmfRepository(user)
