from typing import cast

from . import tortoise_models as ttm
from .ros_pydantic import rmf_fleet_msgs


class BeaconState(rmf_fleet_msgs.BeaconState):
    @staticmethod
    def from_tortoise(tortoise: ttm.BeaconState) -> "BeaconState":
        return BeaconState(
            id=tortoise.id,
            online=cast(bool, tortoise.online),
            category=tortoise.category,
            activated=cast(bool, tortoise.activated),
            level=tortoise.level,
        )

    async def save(self) -> None:
        await ttm.BeaconState.update_or_create(
            {
                "online": self.online,
                "category": self.category,
                "activated": self.activated,
                "level": self.level,
            },
            id=self.id,
        )
