from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary
from rosidl_runtime_py.convert import message_to_ordereddict
from tortoise import fields

from .json_model import json_model


class TaskSummary(json_model(RmfTaskSummary, lambda x: x.task_id)):
    state = fields.IntField()

    ACTIVE_STATES = [
        RmfTaskSummary.STATE_ACTIVE,
        RmfTaskSummary.STATE_PENDING,
        RmfTaskSummary.STATE_QUEUED,
    ]

    @classmethod
    def update_or_create_from_rmf(cls, rmf_msg: RmfTaskSummary):
        id_ = rmf_msg.task_id
        data = message_to_ordereddict(rmf_msg)
        state = rmf_msg.state
        return cls.update_or_create({"data": data, "state": state}, id_=id_)

    def update_from_rmf(self, rmf_msg: RmfTaskSummary):
        super().update_from_rmf(rmf_msg)
        self.state = rmf_msg.state
        return self
