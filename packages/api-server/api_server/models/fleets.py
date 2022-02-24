from typing import Sequence

from tortoise.exceptions import IntegrityError
from tortoise.transactions import in_transaction

from api_server.logger import format_exception, logger

from . import tortoise_models as ttm
from .rmf_api.fleet_log import FleetState as BaseFleetLog
from .rmf_api.fleet_state import FleetState as BaseFleetState
from .rmf_api.log_entry import LogEntry
from .ros_pydantic import rmf_fleet_msgs

RobotMode = rmf_fleet_msgs.RobotMode
Location = rmf_fleet_msgs.Location


class FleetState(BaseFleetState):
    @staticmethod
    def from_db(fleet_state: ttm.FleetState) -> "FleetState":
        return FleetState(**fleet_state.data)

    async def save(self) -> None:
        await ttm.FleetState.update_or_create(
            {
                "data": self.json(),
            },
            name=self.name,
        )


class FleetLog(BaseFleetLog):
    async def _saveFleetLogs(
        self, db_fleet_log: ttm.FleetLog, logs: Sequence[LogEntry]
    ):
        for log in logs:
            await ttm.FleetLogLog.create(
                fleet=db_fleet_log,
                seq=log.seq,
                unix_millis_time=log.unix_millis_time,
                tier=log.tier.name,
                text=log.text,
            )

    async def save(self) -> None:
        async with in_transaction():
            db_fleet_log = (await ttm.FleetLog.get_or_create(name=self.name))[0]
            try:
                if self.log:
                    await self._saveFleetLogs(db_fleet_log, self.log)
            except IntegrityError as e:
                logger.error(format_exception(e))
