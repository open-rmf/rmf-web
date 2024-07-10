from enum import Enum

from tortoise.fields import CharEnumField, CharField
from tortoise.models import Model


class DeliveryAlert(Model):
    """
    Custom alerts for custom delivery tasks
    """

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

    id = CharField(255, pk=True)
    category = CharEnumField(Category, index=True)
    tier = CharEnumField(Tier, index=True)
    task_id = CharField(255, index=True, null=True)
    action = CharEnumField(Action, index=True)
    message = CharField(255, null=True)
