from pydantic import BaseModel, Field

from . import tortoise_models as ttm
from .ros_pydantic import rmf_fleet_msgs

BeaconState = rmf_fleet_msgs.BeaconState


class BeaconState(rmf_fleet_msgs.BeaconState):
    @staticmethod
    def from_tortoise(tortoise: ttm.BeaconState) -> "BeaconState":
        return BeaconState(
            id=tortoise.id,
            online=tortoise.online,
            category=tortoise.category,
            activated=tortoise.activated,
            level=tortoise.level,
        )

    async def save(self) -> None:
        beacon_state, _ = await ttm.BeaconState.update_or_create(
            {
                "online": self.online,
                "category": self.category,
                "activated": self.activated,
                "level": self.level,
            },
            id=self.id,
        )
        if beacon_state is None:
            print(f"Could not save beacon state with ID {beacon_state.id}")
            return
