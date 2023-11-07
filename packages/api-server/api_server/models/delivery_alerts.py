from typing import cast

from . import tortoise_models as ttm
from .ros_pydantic import rmf_fleet_msgs

# TODO(AC): These conversions need to exactly match the enum values defined
# in rmf_fleet_msgs::DeliveryAlert* messages. Any changes to them will require
# the models to be regenerated, and these conversions to be modified.


def category_from_tortoise(
    category: ttm.DeliveryAlert.Category,
) -> rmf_fleet_msgs.DeliveryAlertCategory:
    value = 1
    match (category):
        case ttm.DeliveryAlert.Category.Missing:
            value = 0
        case ttm.DeliveryAlert.Category.Wrong:
            value = 1
        case _:
            pass
    return rmf_fleet_msgs.DeliveryAlertCategory(value=value)


def tier_from_tortoise(
    tier: ttm.DeliveryAlert.Tier,
) -> rmf_fleet_msgs.DeliveryAlertTier:
    value = 1
    match (tier):
        case ttm.DeliveryAlert.Tier.Warning:
            value = 0
        case ttm.DeliveryAlert.Tier.Error:
            value = 1
        case _:
            pass
    return rmf_fleet_msgs.DeliveryAlertTier(value=value)


def action_from_tortoise(
    action: ttm.DeliveryAlert.Action,
) -> rmf_fleet_msgs.DeliveryAlertAction:
    value = 0
    match (action):
        case ttm.DeliveryAlert.Action.Waiting:
            value = 0
        case ttm.DeliveryAlert.Action.Cancel:
            value = 1
        case ttm.DeliveryAlert.Action.Override:
            value = 2
        case ttm.DeliveryAlert.Action.Resume:
            value = 3
        case _:
            pass
    return rmf_fleet_msgs.DeliveryAlertAction(value=value)


def category_from_model(
    category: rmf_fleet_msgs.DeliveryAlertCategory,
) -> ttm.DeliveryAlert.Category:
    value = ttm.DeliveryAlert.Category.Wrong
    match (category.value):
        case 0:
            value = ttm.DeliveryAlert.Category.Missing
        case 1:
            value = ttm.DeliveryAlert.Category.Wrong
        case _:
            pass
    return value


def tier_from_model(tier: rmf_fleet_msgs.DeliveryAlertTier) -> ttm.DeliveryAlert.Tier:
    value = ttm.DeliveryAlert.Tier.Error
    match (tier.value):
        case 0:
            value = ttm.DeliveryAlert.Tier.Warning
        case 1:
            value = ttm.DeliveryAlert.Tier.Error
        case _:
            pass
    return value


def action_from_model(
    action: rmf_fleet_msgs.DeliveryAlertAction,
) -> ttm.DeliveryAlert.Action:
    value = ttm.DeliveryAlert.Action.Waiting
    match (action.value):
        case 0:
            value = ttm.DeliveryAlert.Action.Waiting
        case 1:
            value = ttm.DeliveryAlert.Action.Cancel
        case 2:
            value = ttm.DeliveryAlert.Action.Override
        case 3:
            value = ttm.DeliveryAlert.Action.Resume
    return value


class DeliveryAlert(rmf_fleet_msgs.DeliveryAlert):
    @staticmethod
    def from_tortoise(tortoise: ttm.DeliveryAlert) -> "DeliveryAlert":
        return DeliveryAlert(
            id=tortoise.id,
            category=category_from_tortoise(tortoise.category),
            tier=tier_from_tortoise(tortoise.tier),
            action=action_from_tortoise(tortoise.action),
            task_id=tortoise.task_id,
            message=tortoise.message,
        )

    async def save(self) -> None:
        await ttm.DeliveryAlert.update_or_create(
            {
                "category": category_from_model(self.category),
                "tier": tier_from_model(self.tier),
                "task_id": self.task_id,
                "action": action_from_model(self.action),
                "message": self.message,
            },
            id=self.id,
        )
