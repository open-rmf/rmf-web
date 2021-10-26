from typing import List

from pydantic import BaseModel, Field

from . import tortoise_models as ttm
from .health import basic_health_model
from .ros_pydantic import rmf_building_map_msgs, rmf_lift_msgs
from .ros_pydantic.builtin_interfaces import Time

Lift = rmf_building_map_msgs.Lift
LiftHealth = basic_health_model(ttm.LiftHealth)


class LiftState(rmf_lift_msgs.LiftState):
    available_modes: List[int]

    def __init__(
        self,
        lift_time: Time = Time(),  # builtin_interfaces/Time
        lift_name: str = "",  # string
        available_floors: List[str] = None,  # string
        current_floor: str = "",  # string
        destination_floor: str = "",  # string
        door_state: int = 0,  # uint8
        motion_state: int = 0,  # uint8
        available_modes: List[int] = None,  # uint8
        current_mode: int = 0,  # uint8
        session_id: str = "",  # string
        **kwargs,
    ):
        super().__init__(
            lift_time=lift_time,
            lift_name=lift_name,
            available_floors=available_floors or [],
            current_floor=current_floor,
            destination_floor=destination_floor,
            door_state=door_state,
            motion_state=motion_state,
            available_modes=available_modes or [],
            current_mode=current_mode,
            session_id=session_id,
            **kwargs,
        )

    @staticmethod
    def from_tortoise(tortoise: ttm.LiftState) -> "LiftState":
        return LiftState(**tortoise.data)

    async def save(self) -> None:
        await ttm.LiftState.update_or_create({"data": self.dict()}, id_=self.lift_name)


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
