# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...rmf_dispenser_msgs.msg.DispenserRequestItem import (
    DispenserRequestItem as rmf_dispenser_msgs_msg_DispenserRequestItem,
)
from .Behavior import Behavior as rmf_task_msgs_msg_Behavior


class Delivery(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    task_id: str
    items: list[rmf_dispenser_msgs_msg_DispenserRequestItem]
    pickup_place_name: str
    pickup_dispenser: str
    pickup_behavior: rmf_task_msgs_msg_Behavior
    dropoff_place_name: str
    dropoff_ingestor: str
    dropoff_behavior: rmf_task_msgs_msg_Behavior
