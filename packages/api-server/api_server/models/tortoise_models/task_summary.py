from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary
from tortoise.fields.data import CharField, DatetimeField, IntField
from tortoise.models import Model

from .authorization import ProtectedResource
from .json_mixin import JsonMixin


class TaskSummary(Model, JsonMixin, ProtectedResource):
    fleet_name = CharField(255, null=True, index=True)
    submission_time = DatetimeField(null=True, index=True)
    start_time = DatetimeField(null=True, index=True)
    end_time = DatetimeField(null=True, index=True)
    robot_name = CharField(255, null=True, index=True)
    state = IntField(null=True, index=True)
    task_type = IntField(null=True, index=True)
    priority = IntField(null=True, index=True)

    ACTIVE_STATES = [
        RmfTaskSummary.STATE_ACTIVE,
        RmfTaskSummary.STATE_PENDING,
        RmfTaskSummary.STATE_QUEUED,
    ]
