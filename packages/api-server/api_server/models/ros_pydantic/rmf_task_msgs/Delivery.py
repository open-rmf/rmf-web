# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_dispenser_msgs.DispenserRequestItem import DispenserRequestItem
from ..rmf_task_msgs.Behavior import Behavior


class Delivery(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    task_id: str  # string
    items: list[DispenserRequestItem]  # rmf_dispenser_msgs/DispenserRequestItem
    pickup_place_name: str  # string
    pickup_dispenser: str  # string
    pickup_behavior: Behavior  # rmf_task_msgs/Behavior
    dropoff_place_name: str  # string
    dropoff_ingestor: str  # string
    dropoff_behavior: Behavior  # rmf_task_msgs/Behavior


# # task_id is intended to be a pseudo-random string generated
# # by the caller which can be used to identify this task as it
# # moves between the queues to completion (or failure).
# string task_id
#
# rmf_dispenser_msgs/DispenserRequestItem[] items
#
# string pickup_place_name
# string pickup_dispenser
# Behavior pickup_behavior
#
# string dropoff_place_name
# string dropoff_ingestor
# Behavior dropoff_behavior
