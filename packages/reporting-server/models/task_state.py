from enum import Enum

from tortoise import fields, models
from tortoise.contrib.pydantic import pydantic_model_creator


class TaskStateEnum(str, Enum):
    STATE_ACTIVE = "active"
    STATE_CANCELLED = "cancelled"
    STATE_COMPLETED = "completed"
    STATE_FAILED = "failed"
    STATE_PENDING = "pending"
    STATE_QUEUED = "queued"


class TaskTypeEnum(str, Enum):
    CLEAN = "clean"
    LOOP = "loop"
    DELIVERY = "delivery"


class TaskStateService:
    def get_task_state_name(self, state: int):
        if state == 0:
            return TaskStateEnum.STATE_ACTIVE
        elif state == 1:
            return TaskStateEnum.STATE_CANCELLED
        elif state == 2:
            return TaskStateEnum.STATE_COMPLETED
        elif state == 3:
            return TaskStateEnum.STATE_FAILED
        elif state == 4:
            return TaskStateEnum.STATE_PENDING
        elif state == 5:
            return TaskStateEnum.STATE_QUEUED

    def get_task_type_name(self, type: int):
        if type == 0:
            return TaskTypeEnum.CLEAN
        elif type == 1:
            return TaskTypeEnum.LOOP
        elif type == 2:
            return TaskTypeEnum.DELIVERY


class TaskState(models.Model):
    id = fields.IntField(pk=True)
    created = fields.DatetimeField(auto_now_add=True)
    fleet_name = fields.CharField(max_length=100)
    payload = fields.JSONField()
    task_id = fields.CharField(max_length=100)
    task_state: TaskStateEnum = fields.CharEnumField(
        TaskStateEnum, default=TaskStateEnum.STATE_PENDING
    )
    task_type: TaskTypeEnum = fields.CharEnumField(TaskTypeEnum)
    submission_time = fields.CharField(max_length=200)
    start_time = fields.CharField(max_length=200)
    end_time = fields.CharField(max_length=200)

    service = TaskStateService()


TaskState_Pydantic = pydantic_model_creator(TaskState, name="TaskState")
