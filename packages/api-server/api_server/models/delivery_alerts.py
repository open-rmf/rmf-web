from enum import Enum

from pydantic import BaseModel
from rmf_fleet_msgs.msg import DeliveryAlertAction as RmfDeliveryAlertAction
from rmf_fleet_msgs.msg import DeliveryAlertCategory as RmfDeliveryAlertCateogry
from rmf_fleet_msgs.msg import DeliveryAlertTier as RmfDeliveryAlertTier

# NOTE: These conversions need to exactly match the enum values defined
# in rmf_fleet_msgs::msgs::DeliveryAlert* messages. Any changes to them will
# require these conversions to be modified.


class DeliveryAlert(BaseModel):
    class Category(Enum):
        Missing = RmfDeliveryAlertCateogry.MISSING
        Wrong = RmfDeliveryAlertCateogry.WRONG
        Obstructed = RmfDeliveryAlertCateogry.OBSTRUCTED
        Cancelled = RmfDeliveryAlertCateogry.CANCELLED

    class Tier(Enum):
        Warning = RmfDeliveryAlertTier.WARNING
        Error = RmfDeliveryAlertTier.ERROR

    class Action(Enum):
        Waiting = RmfDeliveryAlertAction.WAITING
        Cancel = RmfDeliveryAlertAction.CANCEL
        Override = RmfDeliveryAlertAction.OVERRIDE
        Resume = RmfDeliveryAlertAction.RESUME

    id: str
    category: Category
    tier: Tier
    action: Action
    task_id: str
    message: str
