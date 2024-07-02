# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_fleet_msgs.DeliveryAlertAction import DeliveryAlertAction
from ..rmf_fleet_msgs.DeliveryAlertCategory import DeliveryAlertCategory
from ..rmf_fleet_msgs.DeliveryAlertTier import DeliveryAlertTier


class DeliveryAlert(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    id: str  # string
    category: DeliveryAlertCategory  # rmf_fleet_msgs/DeliveryAlertCategory
    tier: DeliveryAlertTier  # rmf_fleet_msgs/DeliveryAlertTier
    task_id: str  # string
    action: DeliveryAlertAction  # rmf_fleet_msgs/DeliveryAlertAction
    message: str  # string


# string id
# DeliveryAlertCategory category
# DeliveryAlertTier tier
# string task_id
# DeliveryAlertAction action
# string message
