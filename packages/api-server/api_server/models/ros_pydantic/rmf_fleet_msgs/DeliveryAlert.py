# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.DeliveryAlertAction import DeliveryAlertAction
from ..rmf_fleet_msgs.DeliveryAlertCategory import DeliveryAlertCategory
from ..rmf_fleet_msgs.DeliveryAlertTier import DeliveryAlertTier


class DeliveryAlert(pydantic.BaseModel):
    id: str = ""  # string
    category: DeliveryAlertCategory = (
        DeliveryAlertCategory()
    )  # rmf_fleet_msgs/DeliveryAlertCategory
    tier: DeliveryAlertTier = DeliveryAlertTier()  # rmf_fleet_msgs/DeliveryAlertTier
    task_id: str = ""  # string
    action: DeliveryAlertAction = (
        DeliveryAlertAction()
    )  # rmf_fleet_msgs/DeliveryAlertAction
    message: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "id",
                "category",
                "tier",
                "task_id",
                "action",
                "message",
            ],
        }


# string id
# DeliveryAlertCategory category
# DeliveryAlertTier tier
# string task_id
# DeliveryAlertAction action
# string message
