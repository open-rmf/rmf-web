from enum import Enum

from pydantic import BaseModel
from rmf_fleet_msgs.msg import DeliveryAlertAction as RmfDeliveryAlertAction
from rmf_fleet_msgs.msg import DeliveryAlertCategory as RmfDeliveryAlertCateogry
from rmf_fleet_msgs.msg import DeliveryAlertTier as RmfDeliveryAlertTier

# NOTE: These conversions need to exactly match the enum values defined
# in rmf_fleet_msgs::msgs::DeliveryAlert* messages. Any changes to them will
# require these conversions to be modified.


class DeliveryAlert(BaseModel):
    class Category(str, Enum):
        Missing = "missing"
        Wrong = "wrong"
        Obstructed = "obstructed"
        Cancelled = "cancelled"

        @staticmethod
        def from_rmf_value(value: int) -> "DeliveryAlert.Category":
            match (value):
                case RmfDeliveryAlertCateogry.MISSING:
                    return DeliveryAlert.Category.Missing
                case RmfDeliveryAlertCateogry.WRONG:
                    return DeliveryAlert.Category.Wrong
                case RmfDeliveryAlertCateogry.OBSTRUCTED:
                    return DeliveryAlert.Category.Obstructed
                case RmfDeliveryAlertCateogry.CANCELLED:
                    return DeliveryAlert.Category.Cancelled
                case _:
                    raise ValueError()

        def to_rmf_value(self) -> int:
            match (self.value):
                case DeliveryAlert.Category.Missing:
                    return RmfDeliveryAlertCateogry.MISSING
                case DeliveryAlert.Category.Wrong:
                    return RmfDeliveryAlertCateogry.WRONG
                case DeliveryAlert.Category.Obstructed:
                    return RmfDeliveryAlertCateogry.OBSTRUCTED
                case DeliveryAlert.Category.Cancelled:
                    return RmfDeliveryAlertCateogry.CANCELLED
                case _:
                    raise ValueError()

    class Tier(str, Enum):
        Warning = "warning"
        Error = "error"

        @staticmethod
        def from_rmf_value(value: int) -> "DeliveryAlert.Tier":
            match (value):
                case RmfDeliveryAlertTier.WARNING:
                    return DeliveryAlert.Tier.Warning
                case RmfDeliveryAlertTier.ERROR:
                    return DeliveryAlert.Tier.Error
                case _:
                    raise ValueError()

        def to_rmf_value(self) -> int:
            match (self.value):
                case DeliveryAlert.Tier.Warning:
                    return RmfDeliveryAlertTier.WARNING
                case DeliveryAlert.Tier.Error:
                    return RmfDeliveryAlertTier.ERROR
                case _:
                    raise ValueError()

    class Action(str, Enum):
        Waiting = "waiting"
        Cancel = "cancel"
        Override = "override"
        Resume = "resume"

        @staticmethod
        def from_rmf_value(value: int) -> "DeliveryAlert.Action":
            match (value):
                case RmfDeliveryAlertAction.WAITING:
                    return DeliveryAlert.Action.Waiting
                case RmfDeliveryAlertAction.CANCEL:
                    return DeliveryAlert.Action.Cancel
                case RmfDeliveryAlertAction.OVERRIDE:
                    return DeliveryAlert.Action.Override
                case RmfDeliveryAlertAction.RESUME:
                    return DeliveryAlert.Action.Resume
                case _:
                    raise ValueError()

        def to_rmf_value(self) -> int:
            match (self.value):
                case DeliveryAlert.Action.Waiting:
                    return RmfDeliveryAlertAction.WAITING
                case DeliveryAlert.Action.Cancel:
                    return RmfDeliveryAlertAction.CANCEL
                case DeliveryAlert.Action.Override:
                    return RmfDeliveryAlertAction.OVERRIDE
                case DeliveryAlert.Action.Resume:
                    return RmfDeliveryAlertAction.RESUME
                case _:
                    raise ValueError()

    id: str
    category: Category
    tier: Tier
    action: Action
    task_id: str
    message: str
