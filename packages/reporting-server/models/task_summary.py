from enum import Enum

from tortoise import fields, models
from tortoise.contrib.pydantic import pydantic_model_creator

# INFO:app.BookKeeper.task_summary:{"fleet_name": "tinyRobot", "task_id": "Loop0", "task_profile": {"task_id": "Loop0", "submission_time": {"sec": 131, "nanosec": 553000000}, "description": {"start_time": {"sec": 1623383402, "nanosec": 0}, "priority": {"value": 0}, "task_type": {"type": 1}, "station": {"task_id": "", "robot_type": "", "place_name": ""}, "loop": {"task_id": "", "robot_type": "", "num_loops": 1, "start_name": "supplies", "finish_name": "coe"}, "delivery": {"task_id": "", "items": [], "pickup_place_name": "", "pickup_dispenser": "", "pickup_behavior": {"name": "", "parameters": []}, "dropoff_place_name": "", "dropoff_ingestor": "", "dropoff_behavior": {"name": "", "parameters": []}}, "clean": {"start_waypoint": ""}}}, "state": 0, "status": "", "submission_time": {"sec": 0, "nanosec": 0}, "start_time": {"sec": 1623383362, "nanosec": 348338289}, "end_time": {"sec": 1623383449, "nanosec": 79154833}, "robot_name": "tinyRobot2"}


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


class TaskSummaryService:
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


class Time:
    sec = fields.IntField(pk=True)
    nanosec = fields.IntField(pk=True)


class Priority:
    value = fields.IntField(pk=True)


class TaskType:
    type = TaskTypeEnum = fields.CharEnumField(TaskTypeEnum)


class Station:
    task_id = fields.CharField(max_length=50)
    robot_type = fields.CharField(max_length=50)
    place_name = fields.CharField(max_length=50)


class Loop:
    task_id = fields.CharField(max_length=50)
    robot_type = fields.CharField(max_length=50)
    num_loops = fields.IntField(max_length=50)
    start_name = fields.CharField(max_length=50)
    finish_name = fields.CharField(max_length=50)


class Behavior:
    name = fields.CharField(max_length=50)
    parameters = []


class Delivery:
    task_id = fields.CharField(max_length=50)
    items = []
    pickup_place_name = fields.CharField(max_length=50)
    pickup_dispenser = fields.CharField(max_length=50)
    pickup_behavior: Behavior = fields.JSONField()
    dropoff_place_name = fields.CharField(max_length=50)
    dropoff_ingestor = fields.CharField(max_length=50)
    dropoff_behavior: Behavior = fields.JSONField()


class Clean:
    start_waypoint = fields.CharField(max_length=50)


class Description:
    start_time: Time = fields.JSONField()
    priority: Priority = fields.JSONField()
    task_type: TaskType = fields.JSONField()
    station: Station = fields.JSONField()
    loop: Loop = fields.JSONField()
    delivery: Delivery = fields.JSONField()
    clean: Clean = fields.JSONField()


class TaskProfile:
    task_id = fields.CharField(max_length=50)
    submission_time: Time = fields.JSONField()
    description: Description = fields.JSONField()


class TaskSummary(models.Model):
    id = fields.IntField(pk=True)
    created = fields.DatetimeField(auto_now_add=True)
    payload = fields.JSONField()
    fleet_name = fields.CharField(max_length=50)
    task_id = fields.CharField(max_length=50)
    task_profile: TaskProfile = fields.JSONField()
    state: TaskStateEnum = fields.CharEnumField(
        TaskStateEnum, default=TaskStateEnum.STATE_PENDING
    )
    status: fields.CharField(max_length=100)
    submission_time: Time = fields.JSONField()
    start_time: Time = fields.JSONField()
    end_time: Time = fields.JSONField()
    robot_name = fields.CharField(max_length=50)

    service = TaskSummaryService()


TaskSummary_Pydantic = pydantic_model_creator(TaskSummary, name="TaskSummary")
