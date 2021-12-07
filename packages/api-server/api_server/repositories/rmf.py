from typing import List, Optional, cast

from api_server.models import (
    BuildingMap,
    Dispenser,
    DispenserHealth,
    DispenserState,
    Door,
    DoorHealth,
    DoorState,
    FleetState,
    Ingestor,
    IngestorHealth,
    IngestorState,
    Lift,
    LiftHealth,
    LiftState,
    Pagination,
    RobotHealth,
    User,
)
from api_server.models import tortoise_models as ttm
from api_server.models.fleets import Fleet, Robot
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

    async def get_bulding_map(self) -> Optional[BuildingMap]:
        building_map = await ttm.BuildingMap.first()
        if building_map is None:
            return None
        return BuildingMap(**building_map.data)

    async def get_doors(self) -> List[Door]:
        building_map = await self.get_bulding_map()
        if building_map is None:
            return []
        return [door for level in building_map.levels for door in level.doors]

    async def get_door_state(self, door_name: str) -> Optional[DoorState]:
        door_state = await ttm.DoorState.get_or_none(id_=door_name)
        if door_state is None:
            return None
        return DoorState(**door_state.data)

    async def get_door_health(self, door_name: str) -> Optional[DoorHealth]:
        door_health = await ttm.DoorHealth.get_or_none(id_=door_name)
        if door_health is None:
            return None
        return await DoorHealth.from_tortoise(door_health)

    async def get_lifts(self) -> List[Lift]:
        building_map = await self.get_bulding_map()
        if building_map is None:
            return []
        return building_map.lifts

    async def get_lift_state(self, lift_name: str) -> Optional[LiftState]:
        lift_state = await ttm.LiftState.get_or_none(id_=lift_name)
        if lift_state is None:
            return None
        return LiftState(**lift_state.data)

    async def get_lift_health(self, lift_name: str) -> Optional[LiftHealth]:
        lift_health = await ttm.LiftHealth.get_or_none(id_=lift_name)
        if lift_health is None:
            return None
        return await LiftHealth.from_tortoise(lift_health)

    async def get_dispensers(self) -> List[Dispenser]:
        states = await ttm.DispenserState.all()
        return [Dispenser(guid=state.data["guid"]) for state in states]

    async def get_dispenser_state(self, guid: str) -> Optional[DispenserState]:
        dispenser_state = await ttm.DispenserState.get_or_none(id_=guid)
        if dispenser_state is None:
            return None
        return DispenserState(**dispenser_state.data)

    async def get_dispenser_health(self, guid: str) -> Optional[DispenserHealth]:
        dispenser_health = await ttm.DispenserHealth.get_or_none(id_=guid)
        if dispenser_health is None:
            return None
        return await DispenserHealth.from_tortoise(dispenser_health)

    async def get_ingestors(self) -> List[Ingestor]:
        states = await ttm.IngestorState.all()
        return [Ingestor(guid=state.data["guid"]) for state in states]

    async def get_ingestor_state(self, guid: str) -> Optional[IngestorState]:
        ingestor_state = await ttm.IngestorState.get_or_none(id_=guid)
        if ingestor_state is None:
            return None
        return IngestorState(**ingestor_state.data)

    async def get_ingestor_health(self, guid: str) -> Optional[IngestorHealth]:
        ingestor_health = await ttm.IngestorHealth.get_or_none(id_=guid)
        if ingestor_health is None:
            return None
        return await IngestorHealth.from_tortoise(ingestor_health)

    async def query_fleets(
        self, pagination: Pagination, *, fleet_name: Optional[str] = None
    ) -> List[Fleet]:
        filter_params = {}
        if fleet_name is not None:
            filter_params["id___in"] = fleet_name.split(",")
        states = await add_pagination(
            ttm.FleetState.filter(**filter_params), pagination, {"fleet_name": "id_"}
        )
        return [Fleet(name=s.id_, state=FleetState.from_tortoise(s)) for s in states]

    async def get_fleet_state(self, fleet_name: str) -> Optional[FleetState]:
        fleet_state = await ttm.FleetState.get_or_none(id_=fleet_name)
        if fleet_state is None:
            return None
        return FleetState(**fleet_state.data)

    async def query_robots(
        self,
        pagination: Pagination,
        *,
        fleet_name: Optional[str] = None,
        robot_name: Optional[str] = None,
    ) -> List[Robot]:
        filter_params = {}
        if fleet_name is not None:
            filter_params["fleet_name__in"] = fleet_name.split(",")
        if robot_name is not None:
            filter_params["robot_name__in"] = robot_name.split(",")

        robot_states = await add_pagination(
            ttm.RobotState.filter(**filter_params), pagination
        )
        return [
            Robot(fleet=r.fleet_name, name=r.robot_name, state=r.data)
            for r in robot_states
        ]

    async def get_robot_health(
        self, fleet_name: str, robot_name: str
    ) -> Optional[RobotHealth]:
        robot_health = await ttm.RobotHealth.get_or_none(
            id_=f"{fleet_name}/{robot_name}"
        )
        if robot_health is None:
            return None
        return await RobotHealth.from_tortoise(robot_health)

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
