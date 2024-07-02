from typing import List, Optional, cast

from fastapi import Depends

from api_server.authenticator import user_dep
from api_server.models import (
    BuildingMap,
    Dispenser,
    DispenserState,
    Door,
    DoorState,
    Ingestor,
    IngestorState,
    Lift,
    LiftState,
    Pagination,
    User,
)
from api_server.models import tortoise_models as ttm
from api_server.query import add_pagination


class RmfRepository:
    def __init__(self, user: User = Depends(user_dep)):
        self.user = user

    @staticmethod
    def _build_filter_params(**queries: dict):
        filter_params = {}
        for k, v in queries.items():
            if v is not None:
                filter_params[k] = v
        return filter_params

    async def get_bulding_map(self) -> Optional[BuildingMap]:
        building_map = await ttm.BuildingMap.first()
        if building_map is None:
            return None
        return BuildingMap.model_construct(**cast(dict, building_map.data))

    async def get_doors(self) -> List[Door]:
        building_map = await self.get_bulding_map()
        if building_map is None:
            return []
        return [door for level in building_map.levels for door in level.doors]

    async def get_door_state(self, door_name: str) -> Optional[DoorState]:
        door_state = await ttm.DoorState.get_or_none(id_=door_name)
        if door_state is None:
            return None
        return DoorState.model_construct(**cast(dict, door_state.data))

    async def get_lifts(self) -> List[Lift]:
        building_map = await self.get_bulding_map()
        if building_map is None:
            return []
        return building_map.lifts

    async def get_lift_state(self, lift_name: str) -> Optional[LiftState]:
        lift_state = await ttm.LiftState.get_or_none(id_=lift_name)
        if lift_state is None:
            return None
        return LiftState.model_construct(**cast(dict, lift_state.data))

    async def get_dispensers(self) -> List[Dispenser]:
        states = await ttm.DispenserState.all()
        return [
            Dispenser.model_construct(guid=cast(dict, state.data)["guid"])
            for state in states
        ]

    async def get_dispenser_state(self, guid: str) -> Optional[DispenserState]:
        dispenser_state = await ttm.DispenserState.get_or_none(id_=guid)
        if dispenser_state is None:
            return None
        return DispenserState.model_construct(**cast(dict, dispenser_state.data))

    async def get_ingestors(self) -> List[Ingestor]:
        states = await ttm.IngestorState.all()
        return [
            Ingestor.model_construct(guid=cast(dict, state.data)["guid"])
            for state in states
        ]

    async def get_ingestor_state(self, guid: str) -> Optional[IngestorState]:
        ingestor_state = await ttm.IngestorState.get_or_none(id_=guid)
        if ingestor_state is None:
            return None
        return IngestorState.model_construct(**cast(dict, ingestor_state.data))

    async def query_users(
        self,
        pagination: Pagination,
        *,
        username: Optional[str] = None,
        is_admin: Optional[bool] = None,
    ) -> List[str]:
        filter_params = {}
        if username is not None:
            filter_params["username__istartswith"] = username
        if is_admin is not None:
            filter_params["is_admin"] = is_admin
        return cast(
            List[str],
            await add_pagination(
                ttm.User.filter(**filter_params),
                pagination,
            ).values_list("username", flat=True),
        )
