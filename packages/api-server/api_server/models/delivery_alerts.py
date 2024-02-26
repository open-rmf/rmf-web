from enum import Enum

from pydantic import BaseModel

# NOTE: These conversions need to exactly match the enum values defined
# in rmf_fleet_msgs::msgs::DeliveryAlert* messages. Any changes to them will
# require these conversions to be modified.


def category_from_msg(category: int) -> str:
    value = DeliveryAlert.Category.Wrong
    match (category):
        case 0:
            value = DeliveryAlert.Category.Missing
        case 1:
            value = DeliveryAlert.Category.Wrong
        case 2:
            value = DeliveryAlert.Category.Obstructed
        case 3:
            value = DeliveryAlert.Category.Cancelled
        case _:
            pass
    return value


def tier_from_msg(tier: int) -> str:
    value = DeliveryAlert.Tier.Error
    match (tier):
        case 0:
            value = DeliveryAlert.Tier.Warning
        case 1:
            value = DeliveryAlert.Tier.Error
        case _:
            pass
    return value


def action_from_msg(action: int) -> str:
    value = DeliveryAlert.Action.Waiting
    match (action):
        case 0:
            value = DeliveryAlert.Action.Waiting
        case 1:
            value = DeliveryAlert.Action.Cancel
        case 2:
            value = DeliveryAlert.Action.Override
        case 3:
            value = DeliveryAlert.Action.Resume
    return value


def category_to_msg(category: str) -> int:
    value = 1
    match (category):
        case DeliveryAlert.Category.Missing:
            value = 0
        case DeliveryAlert.Category.Wrong:
            value = 1
        case DeliveryAlert.Category.Obstructed:
            value = 2
        case DeliveryAlert.Category.Cancelled:
            value = 3
        case _:
            pass
    return value


def tier_to_msg(tier: str) -> int:
    value = 1
    match (tier):
        case DeliveryAlert.Tier.Warning:
            value = 0
        case DeliveryAlert.Tier.Error:
            value = 1
        case _:
            pass
    return value


def action_to_msg(action: str) -> int:
    value = 0
    match (action):
        case DeliveryAlert.Action.Waiting:
            value = 0
        case DeliveryAlert.Action.Cancel:
            value = 1
        case DeliveryAlert.Action.Override:
            value = 2
        case DeliveryAlert.Action.Resume:
            value = 3
        case _:
            pass
    return value


class DeliveryAlert(BaseModel):
    class Category(str, Enum):
        Missing = "missing"
        Wrong = "wrong"
        Obstructed = "obstructed"
        Cancelled = "cancelled"

    class Tier(str, Enum):
        Warning = "warning"
        Error = "error"

    class Action(str, Enum):
        Waiting = "waiting"
        Cancel = "cancelled"
        Override = "override"
        Resume = "resume"

    id: str
    category: Category
    tier: Tier
    action: Action
    task_id: str
    message: str
