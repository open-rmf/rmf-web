from datetime import datetime
from typing import Annotated, Sequence

from fastapi import Depends
from tortoise.exceptions import IntegrityError
from tortoise.query_utils import Prefetch
from tortoise.transactions import in_transaction

from api_server.authenticator import user_dep
from api_server.logging import LoggerAdapter, get_logger
from api_server.models import FleetLog, FleetState, LogEntry, User
from api_server.models import tortoise_models as ttm


class FleetRepository:
    def __init__(
        self,
        user: Annotated[User, Depends(user_dep)],
        logger: Annotated[LoggerAdapter, Depends(get_logger)],
    ):
        self.user = user
        self.logger = logger

    async def get_all_fleets(self) -> list[FleetState]:
        db_states = await ttm.FleetState.all().values_list("data")
        return [FleetState(**s[0]) for s in db_states]

    async def get_fleet_state(self, name: str) -> FleetState | None:
        # TODO: enforce with authz
        result = await ttm.FleetState.get_or_none(name=name)
        if result is None:
            return None
        return FleetState.model_validate(result.data)

    async def get_fleet_log(
        self, name: str, between: tuple[datetime, datetime] | None
    ) -> FleetLog | None:
        """
        :param between: The period in unix millis to fetch.
        """
        if between:
            between_filters = {
                "unix_millis_time__gte": between[0].timestamp() * 1000,
                "unix_millis_time__lte": between[1].timestamp() * 1000,
            }
        else:
            between_filters = {}
        result = await ttm.FleetLog.get_or_none(name=name).prefetch_related(
            Prefetch(
                "log",
                ttm.FleetLogLog.filter(**between_filters),
            ),
            Prefetch("robots__log", ttm.FleetLogRobotsLog.filter(**between_filters)),
        )
        if result is None:
            return None
        robots = {}
        for db_robot in result.robots:
            robot = [LogEntry(**dict(db_log)) for db_log in db_robot.log]
            robots[db_robot.name] = robot
        return FleetLog(
            name=result.name,
            log=[LogEntry(**dict(db_log)) for db_log in result.log],
            robots=robots,
        )

    async def save_fleet_state(self, fleet_state: FleetState) -> None:
        await ttm.FleetState.update_or_create(
            {
                "data": fleet_state.model_dump_json(),
            },
            name=fleet_state.name,
        )

    async def save_fleet_log(self, fleet_log: FleetLog) -> None:
        async def _save_logs(db_fleet_log: ttm.FleetLog, logs: Sequence[LogEntry]):
            for log in logs:
                await ttm.FleetLogLog.create(
                    fleet=db_fleet_log,
                    seq=log.seq,
                    unix_millis_time=log.unix_millis_time,
                    tier=log.tier.name,
                    text=log.text,
                )

        async with in_transaction():
            db_fleet_log = (await ttm.FleetLog.get_or_create(name=fleet_log.name))[0]
            try:
                if fleet_log.log:
                    await _save_logs(db_fleet_log, fleet_log.log)
            except IntegrityError as e:
                self.logger.error(e)
