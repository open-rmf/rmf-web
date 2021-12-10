from .rmf_api.fleet_state import FleetState as BaseFleetState
from .ros_pydantic import rmf_fleet_msgs
from .tortoise_models import FleetState as DbFleetState

RobotMode = rmf_fleet_msgs.RobotMode
Location = rmf_fleet_msgs.Location


class FleetState(BaseFleetState):
    @staticmethod
    def from_db(fleet_state: DbFleetState) -> "FleetState":
        return FleetState(**fleet_state.data)

    async def save(self) -> None:
        await DbFleetState.update_or_create(
            {
                "data": self.json(),
            },
            name=self.name,
        )
