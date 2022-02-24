from typing import List, Optional, Tuple, cast

from fastapi import Depends
from tortoise.query_utils import Prefetch

from api_server.authenticator import user_dep
from api_server.models import FleetLog, FleetState, LogEntry, User
from api_server.models import tortoise_models as ttm


class FleetRepository:
    def __init__(self, user: User):
        self.user = user

    async def get_all_fleets(self) -> List[FleetState]:
        db_states = await ttm.FleetState.all().values_list("data", flat=True)
        return [FleetState(**s) for s in db_states]

    async def get_fleet_state(self, name: str) -> Optional[FleetState]:
        # TODO: enforce with authz
        result = await ttm.FleetState.get_or_none(name=name)
        if result is None:
            return None
        return FleetState(**result.data)

    async def get_fleet_log(
        self, name: str, between: Tuple[int, int]
    ) -> Optional[FleetLog]:
        """
        :param between: The period in unix millis to fetch.
        """
        between_filters = {
            "unix_millis_time__gte": between[0],
            "unix_millis_time__lte": between[1],
        }
        result = cast(
            Optional[ttm.FleetLog],
            await ttm.FleetLog.get_or_none(name=name).prefetch_related(
                Prefetch(
                    "log",
                    ttm.FleetLogLog.filter(**between_filters),
                ),
                Prefetch(
                    "robots__log", ttm.FleetLogRobotsLog.filter(**between_filters)
                ),
            ),
        )
        if result is None:
            return None
        robots = {}
        for db_robot in result.robots:
            robot = [LogEntry(**dict(db_log)) for db_log in db_robot.log]
            robots[db_robot.name] = robot
        return FleetLog.construct(
            name=result.name,
            log=[LogEntry(**dict(db_log)) for db_log in result.log],
            robots=robots,
        )


def fleet_repo_dep(user: User = Depends(user_dep)):
    return FleetRepository(user)
