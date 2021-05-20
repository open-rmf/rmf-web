from tortoise import Model

from ..fleets import FleetState as PydanticFleetState
from .json_mixin import JsonMixin


class FleetState(Model, JsonMixin):
    @staticmethod
    async def save_pydantic(fleet_state: PydanticFleetState):
        await FleetState.update_or_create(
            {
                "data": fleet_state.dict(),
            },
            id_=fleet_state.name,
        )

    def to_pydantic(self):
        return PydanticFleetState(**self.data)
