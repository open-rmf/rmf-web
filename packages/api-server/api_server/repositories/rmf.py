from typing import Annotated, Literal, cast

from fastapi import Depends
from tortoise.queryset import ValuesListQuery

from api_server.authenticator import user_dep
from api_server.models import (
    BeaconState,
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
    def __init__(self, user: Annotated[User, Depends(user_dep)]):
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
        return BuildingMap.model_validate(building_map.data)

    async def save_building_map(self, building_map: BuildingMap) -> None:
        existing_maps = await ttm.BuildingMap.all()
        for m in existing_maps:
            if m.id_ != building_map.name:
                await m.delete()
        await ttm.BuildingMap.update_or_create(
            {"data": building_map.model_dump()}, id_=building_map.name
        )

    async def get_doors(self) -> list[Door]:
        building_map = await self.get_bulding_map()
        if building_map is None:
            return []
        return [door for level in building_map.levels for door in level.doors]

    async def get_door_state(self, door_name: str) -> DoorState | None:
        door_state = await ttm.DoorState.get_or_none(id_=door_name)
        if door_state is None:
            return None
        return DoorState.model_validate(door_state.data)

    async def save_door_state(self, door_state: DoorState) -> None:
        await ttm.DoorState.update_or_create(
            {"data": door_state.model_dump()}, id_=door_state.door_name
        )

    async def get_lifts(self) -> list[Lift]:
        building_map = await self.get_bulding_map()
        if building_map is None:
            return []
        return building_map.lifts

    async def get_lift_state(self, lift_name: str) -> LiftState | None:
        lift_state = await ttm.LiftState.get_or_none(id_=lift_name)
        if lift_state is None:
            return None
        return LiftState.model_validate(lift_state.data)

    async def save_lift_state(self, lift_state: LiftState) -> None:
        await ttm.LiftState.update_or_create(
            {"data": lift_state.model_dump()}, id_=lift_state.lift_name
        )

    async def get_dispensers(self) -> list[Dispenser]:
        states = await ttm.DispenserState.all()
        return [Dispenser.model_validate(state.data) for state in states]

    async def save_dispenser_state(self, dispenser_state: DispenserState) -> None:
        await ttm.DispenserState.update_or_create(
            {"data": dispenser_state.model_dump()}, id_=dispenser_state.guid
        )

    async def get_dispenser_state(self, guid: str) -> DispenserState | None:
        dispenser_state = await ttm.DispenserState.get_or_none(id_=guid)
        if dispenser_state is None:
            return None
        return DispenserState.model_validate(dispenser_state.data)

    async def get_ingestors(self) -> list[Ingestor]:
        states = await ttm.IngestorState.all()
        return [Ingestor.model_validate(state.data) for state in states]

    async def save_ingestor_state(self, ingestor_state: IngestorState) -> None:
        await ttm.IngestorState.update_or_create(
            {"data": ingestor_state.model_dump()}, id_=ingestor_state.guid
        )

    async def get_ingestor_state(self, guid: str) -> IngestorState | None:
        ingestor_state = await ttm.IngestorState.get_or_none(id_=guid)
        if ingestor_state is None:
            return None
        return IngestorState.model_validate(ingestor_state.data)

    async def save_beacon_state(self, beacon_state: BeaconState) -> None:
        d = beacon_state.model_dump()
        del d["id"]
        await ttm.BeaconState.update_or_create(d, id_=beacon_state.id)

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
        return await cast(
            ValuesListQuery[Literal[True]],
            add_pagination(
                ttm.User.filter(**filter_params),
                pagination,
            ).values_list("username", flat=True),
        )
