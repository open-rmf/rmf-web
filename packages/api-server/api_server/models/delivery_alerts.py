# pyright: reportGeneralTypeIssues=false

from . import tortoise_models as ttm

# TODO(AC): These conversions need to exactly match the enum values defined
# in rmf_fleet_msgs::msgs::DeliveryAlert* messages. Any changes to them will
# require these conversions to be modified.


def category_from_msg(category: int) -> str:
    value = ttm.DeliveryAlert.Category.Wrong
    match (category):
        case 0:
            value = ttm.DeliveryAlert.Category.Missing
        case 1:
            value = ttm.DeliveryAlert.Category.Wrong
        case 2:
            value = ttm.DeliveryAlert.Category.Obstructed
        case 3:
            value = ttm.DeliveryAlert.Category.Cancelled
        case _:
            pass
    return value


def tier_from_msg(tier: int) -> str:
    value = ttm.DeliveryAlert.Tier.Error
    match (tier):
        case 0:
            value = ttm.DeliveryAlert.Tier.Warning
        case 1:
            value = ttm.DeliveryAlert.Tier.Error
        case _:
            pass
    return value


def action_from_msg(action: int) -> str:
    value = ttm.DeliveryAlert.Action.Waiting
    match (action):
        case 0:
            value = ttm.DeliveryAlert.Action.Waiting
        case 1:
            value = ttm.DeliveryAlert.Action.Cancel
        case 2:
            value = ttm.DeliveryAlert.Action.Override
        case 3:
            value = ttm.DeliveryAlert.Action.Resume
    return value


def category_to_msg(category: str) -> int:
    value = 1
    match (category):
        case ttm.DeliveryAlert.Category.Missing:
            value = 0
        case ttm.DeliveryAlert.Category.Wrong:
            value = 1
        case ttm.DeliveryAlert.Category.Obstructed:
            value = 2
        case ttm.DeliveryAlert.Category.Cancelled:
            value = 3
        case _:
            pass
    return value


def tier_to_msg(tier: str) -> int:
    value = 1
    match (tier):
        case ttm.DeliveryAlert.Tier.Warning:
            value = 0
        case ttm.DeliveryAlert.Tier.Error:
            value = 1
        case _:
            pass
    return value


def action_to_msg(action: str) -> int:
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
    return value


class DeliveryAlert(ttm.DeliveryAlertPydantic):
    @staticmethod
    def from_tortoise(tortoise: ttm.DeliveryAlert) -> "DeliveryAlert":
        return DeliveryAlert(
            id=tortoise.id,
            category=tortoise.category,
            tier=tortoise.tier,
            action=tortoise.action,
            task_id=tortoise.task_id,
            message=tortoise.message,
        )

    async def save(self) -> None:
        # Get previous delivery alert for this task ID
        prev_waiting_delivery_alert = await ttm.DeliveryAlert.get_or_none(
            task_id=self.task_id, action="waiting"
        )

        await ttm.DeliveryAlert.update_or_create(
            {
                "category": self.category,
                "tier": self.tier,
                "task_id": self.task_id,
                "action": self.action,
                "message": self.message,
            },
            id=self.id,
        )

        # If there was a previous delivery alert for this task ID, we cancel it
        if prev_waiting_delivery_alert is not None:
            await prev_waiting_delivery_alert.update_from_dict({"action": "cancelled"})
            await prev_waiting_delivery_alert.save()
