from enum import Enum
from typing import Optional, Union

from pydantic import BaseModel, validator
from rmf_task_msgs.msg import TaskType as RmfTaskType

from .ros_pydantic.rmf_task_msgs.TaskSummary import TaskSummary as PydanticTaskSummary


class CleanTaskDescription(BaseModel):
    cleaning_zone: str


class LoopTaskDescription(BaseModel):
    num_loops: int
    start_name: str
    finish_name: str


class DeliveryTaskDescription(BaseModel):
    pickup_place_name: str
    pickup_dispenser: str
    dropoff_ingestor: str
    dropoff_place_name: str


TaskDescriptionT = Union[
    CleanTaskDescription, LoopTaskDescription, DeliveryTaskDescription
]


class TaskTypeEnum(Enum):
    CLEAN = RmfTaskType.TYPE_CLEAN
    LOOP = RmfTaskType.TYPE_LOOP
    DELIVERY = RmfTaskType.TYPE_DELIVERY


class Task(BaseModel):
    task_summary: PydanticTaskSummary
    progress: str


class SubmitTask(BaseModel):
    task_type: TaskTypeEnum
    start_time: int
    priority: Optional[int] = None
    description: TaskDescriptionT

    # pylint: disable=no-self-argument,no-self-use
    @validator("description")
    def description_matches_task_type(cls, description, values):
        if not "task_type" in values:
            return None
        task_type: TaskTypeEnum = values["task_type"]
        if task_type == TaskTypeEnum.CLEAN:
            if not isinstance(description, CleanTaskDescription):
                return CleanTaskDescription.validate(description)
        elif task_type == TaskTypeEnum.LOOP:
            if not isinstance(description, LoopTaskDescription):
                raise TypeError("expected description to be LoopTaskDescription")
        elif task_type == TaskTypeEnum.DELIVERY:
            if not isinstance(description, DeliveryTaskDescription):
                raise TypeError("expected description to be DeliveryTaskDescription")
        return description


class SubmitTaskResponse(BaseModel):
    task_id: str


class CancelTask(BaseModel):
    task_id: str
