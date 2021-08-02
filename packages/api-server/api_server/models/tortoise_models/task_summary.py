from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary
from tortoise import Model, fields

from api_server.models.tortoise_models.authorization import ProtectedResource
from api_server.models.tortoise_models.json_mixin import JsonMixin


class TaskSummary(Model, JsonMixin, ProtectedResource):
    fleet_name = fields.CharField(255, null=True, index=True)
    submission_time = fields.DatetimeField(null=True, index=True)
    start_time = fields.DatetimeField(null=True, index=True)
    end_time = fields.DatetimeField(null=True, index=True)
    robot_name = fields.CharField(255, null=True, index=True)
    state = fields.IntField(null=True, index=True)
    task_type = fields.IntField(null=True, index=True)
    priority = fields.IntField(null=True, index=True)

    ACTIVE_STATES = [
        RmfTaskSummary.STATE_ACTIVE,
        RmfTaskSummary.STATE_PENDING,
        RmfTaskSummary.STATE_QUEUED,
    ]
