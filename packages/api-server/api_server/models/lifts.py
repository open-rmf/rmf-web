from pydantic import BaseModel, Field

from . import tortoise_models as ttm
from .ros_pydantic import rmf_building_map_msgs, rmf_lift_msgs

Lift = rmf_building_map_msgs.Lift


class LiftState(rmf_lift_msgs.LiftState):
    available_modes: list[int]  # pyright: ignore [reportIncompatibleVariableOverride]

    @staticmethod
    def from_tortoise(tortoise: ttm.LiftState) -> "LiftState":
        return LiftState(**tortoise.data)

    async def save(self) -> None:
        await ttm.LiftState.update_or_create(
            {"data": self.model_dump()}, id_=self.lift_name
        )


class LiftRequest(BaseModel):
    request_type: int = Field(
        ...,
        description="https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg",  # pylint: disable=line-too-long
    )
    door_mode: int = Field(
        ...,
        description="https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg",  # pylint: disable=line-too-long
    )
    destination: str
    additional_session_ids: list[str] = Field(
        default=[],
        description="By default the node name of the API server is used, this field allows publishing the same request to additional session IDs",  # pylint: disable=line-too-long
    )
