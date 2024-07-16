# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .DeliveryAlertAction import (
    DeliveryAlertAction as rmf_fleet_msgs_msg_DeliveryAlertAction,
)
from .DeliveryAlertCategory import (
    DeliveryAlertCategory as rmf_fleet_msgs_msg_DeliveryAlertCategory,
)
from .DeliveryAlertTier import DeliveryAlertTier as rmf_fleet_msgs_msg_DeliveryAlertTier


class DeliveryAlert(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    id: str
    category: rmf_fleet_msgs_msg_DeliveryAlertCategory
    tier: rmf_fleet_msgs_msg_DeliveryAlertTier
    task_id: str
    action: rmf_fleet_msgs_msg_DeliveryAlertAction
    message: str
