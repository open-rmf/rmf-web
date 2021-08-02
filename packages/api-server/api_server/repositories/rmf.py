from datetime import datetime
from typing import Dict, List, Optional, TypeVar

from fastapi.exceptions import HTTPException
from tortoise.queryset import QuerySet

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
    TaskStateEnum,
    TaskSummary,
    TaskTypeEnum,
    User,
)
from api_server.models import tortoise_models as ttm
from api_server.models.fleets import Fleet, Robot
from api_server.permissions import Enforcer, RmfAction

T = TypeVar("T")


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

    @staticmethod
    def _add_pagination(
        query: QuerySet[T],
        pagination: Pagination,
        field_mappings: Dict[str, str] = None,
    ) -> QuerySet[T]:
        """
        :param field_mapping: A dict mapping the order fields to the fields used to build the
            query. e.g. a url of `?order_by=order_field` and a field mapping of `{"order_field": "db_field"}`
            will order the query result according to `db_field`.
        """
        field_mappings = field_mappings or {}
        query = query.limit(pagination.limit).offset(pagination.offset)
        if pagination.order_by is not None:
            order_fields = []
            order_values = pagination.order_by.split(",")
            for v in order_values:
                if v[0] in ["-", "+"]:
                    stripped = v[1:]
                    order_fields.append(v[0] + field_mappings.get(stripped, stripped))
                else:
                    order_fields.append(field_mappings.get(v, v))
            query = query.order_by(*order_fields)
        return query

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
        return await ttm.DoorHealth.get_or_none(id_=door_name)

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
        return await ttm.LiftHealth.get_or_none(id_=lift_name)

    async def get_dispensers(self) -> List[Dispenser]:
        states = await ttm.DispenserState.all()
        return [Dispenser(guid=state.data["guid"]) for state in states]

    async def get_dispenser_state(self, guid: str) -> Optional[DispenserState]:
        dispenser_state = await ttm.DispenserState.get_or_none(id_=guid)
        if dispenser_state is None:
            return None
        return DispenserState(**dispenser_state.data)

    async def get_dispenser_health(self, guid: str) -> Optional[DispenserHealth]:
        return await ttm.DispenserHealth.get_or_none(id_=guid)

    async def get_ingestors(self) -> List[Ingestor]:
        states = await ttm.IngestorState.all()
        return [Ingestor(guid=state.data["guid"]) for state in states]

    async def get_ingestor_state(self, guid: str) -> Optional[IngestorState]:
        ingestor_state = await ttm.IngestorState.get_or_none(id_=guid)
        if ingestor_state is None:
            return None
        return IngestorState(**ingestor_state.data)

    async def get_ingestor_health(self, guid: str) -> Optional[IngestorHealth]:
        return await ttm.IngestorHealth.get_or_none(id_=guid)

    async def query_fleets(
        self, pagination: Pagination, *, fleet_name: Optional[str] = None
    ) -> List[Fleet]:
        filter_params = {}
        if fleet_name is not None:
            filter_params["id___in"] = fleet_name.split(",")
        states = await self._add_pagination(
            ttm.FleetState.filter(**filter_params), pagination, {"fleet_name": "id_"}
        )
        return [Fleet(name=s.id_, state=s) for s in states]

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

        robot_states = await self._add_pagination(
            ttm.RobotState.filter(**filter_params), pagination
        )
        return [
            Robot(fleet=r.fleet_name, name=r.robot_name, state=r.data)
            for r in robot_states
        ]

    async def get_robot_health(
        self, fleet_name: str, robot_name: str
    ) -> Optional[RobotHealth]:
        return await ttm.RobotHealth.get_or_none(id_=f"{fleet_name}/{robot_name}")

    async def get_task_summary(self, task_id: str) -> TaskSummary:
        # FIXME: This would fail if task_id contains "_/"
        task_id = task_id.replace("__", "/")
        ts = await Enforcer.query(
            self.user, ttm.TaskSummary, RmfAction.TaskRead
        ).get_or_none(id_=task_id)

        if ts is None:
            raise HTTPException(404)
        return TaskSummary.from_tortoise(ts)

    async def query_task_summaries(
        self,
        pagination: Pagination,
        *,
        task_id: Optional[str] = None,
        fleet_name: Optional[str] = None,
        submission_time_since: Optional[datetime] = None,
        start_time_since: Optional[datetime] = None,
        end_time_since: Optional[datetime] = None,
        robot_name: Optional[str] = None,
        state: Optional[str] = None,
        task_type: Optional[str] = None,
        priority: Optional[int] = None,
    ) -> List[TaskSummary]:
        filter_params = {}
        if task_id is not None:
            filter_params["id___in"] = task_id.split(",")
        if fleet_name is not None:
            filter_params["fleet_name__in"] = fleet_name.split(",")
        if submission_time_since is not None:
            filter_params["submission_time__gte"] = submission_time_since
        if start_time_since is not None:
            filter_params["start_time__gte"] = start_time_since
        if end_time_since is not None:
            filter_params["end_time__gte"] = end_time_since
        if robot_name is not None:
            filter_params["robot_name__in"] = robot_name.split(",")
        if state is not None:
            try:
                filter_params["state__in"] = [
                    TaskStateEnum[s.upper()].value for s in state.split(",")
                ]
            except KeyError as e:
                raise HTTPException(422, "unknown state") from e
        if task_type is not None:
            try:
                filter_params["task_type__in"] = [
                    TaskTypeEnum[t.upper()].value for t in task_type.split(",")
                ]
            except KeyError as e:
                raise HTTPException(422, "unknown task type") from e
        if priority is not None:
            filter_params["priority"] = priority

        q = self._add_pagination(
            ttm.TaskSummary.filter(**filter_params), pagination, {"task_id": "id_"}
        )

        tasks = await Enforcer.query(self.user, q, RmfAction.TaskRead)
        return [TaskSummary.from_tortoise(t) for t in tasks]

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
        return await self._add_pagination(
            ttm.User.filter(**filter_params),
            pagination,
        ).values_list("username", flat=True)
