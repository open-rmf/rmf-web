from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary
from tortoise import Model, fields

from .json_mixin import JsonMixin


class TaskSummary(Model, JsonMixin):
    state = fields.IntField()

    ACTIVE_STATES = [
        RmfTaskSummary.STATE_ACTIVE,
        RmfTaskSummary.STATE_PENDING,
        RmfTaskSummary.STATE_QUEUED,
    ]
