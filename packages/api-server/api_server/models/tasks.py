from enum import IntEnum
from typing import Optional, Union

from pydantic import BaseModel, validator
from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary
from rmf_task_msgs.msg import TaskType as RmfTaskType

from api_server.ros_time import ros_to_py_datetime

from . import tortoise_models as ttm
from .ros_pydantic import builtin_interfaces, rmf_task_msgs


class TaskSummary(rmf_task_msgs.TaskSummary):
    authz_grp: Optional[str] = None

    # pylint: disable=useless-super-delegation
    def __init__(
        self,
        *,
        authz_grp: str = "",
        fleet_name: str = "",  # string
        task_id: str = "",  # string
        task_profile: rmf_task_msgs.TaskProfile = rmf_task_msgs.TaskProfile(),  # rmf_task_msgs/TaskProfile
        state: int = 0,  # uint32
        status: str = "",  # string
        submission_time: builtin_interfaces.Time = builtin_interfaces.Time(),  # builtin_interfaces/Time
        start_time: builtin_interfaces.Time = builtin_interfaces.Time(),  # builtin_interfaces/Time
        end_time: builtin_interfaces.Time = builtin_interfaces.Time(),  # builtin_interfaces/Time
        robot_name: str = "",  # string
        **kwargs,
    ):
        super().__init__(
            authz_grp=authz_grp,
            fleet_name=fleet_name,
            task_id=task_id,
            task_profile=task_profile,
            state=state,
            status=status,
            submission_time=submission_time,
            start_time=start_time,
            end_time=end_time,
            robot_name=robot_name,
            **kwargs,
        )

    @staticmethod
    def from_tortoise(tortoise: ttm.TaskSummary) -> "TaskSummary":
        return TaskSummary(authz_grp=tortoise.authz_grp, **tortoise.data)

    async def save(self, authz_grp: Optional[str] = None):
        dic = self.dict()
        del dic["authz_grp"]

        defaults = {
            "fleet_name": self.fleet_name,
            "submission_time": ros_to_py_datetime(self.submission_time),
            "start_time": ros_to_py_datetime(self.start_time),
            "end_time": ros_to_py_datetime(self.end_time),
            "robot_name": self.robot_name,
            "state": self.state,
            "task_type": self.task_profile.description.task_type.type,
            "priority": self.task_profile.description.priority.value,
            "data": dic,
        }
        if authz_grp is not None:
            defaults["authz_grp"] = authz_grp
        await ttm.TaskSummary.update_or_create(defaults, id_=self.task_id)


class TaskStateEnum(IntEnum):
    ACTIVE = RmfTaskSummary.STATE_ACTIVE
    CANCELLED = RmfTaskSummary.STATE_CANCELED
    COMPLETED = RmfTaskSummary.STATE_COMPLETED
    FAILED = RmfTaskSummary.STATE_FAILED
    PENDING = RmfTaskSummary.STATE_PENDING
    QUEUED = RmfTaskSummary.STATE_QUEUED


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


class TaskTypeEnum(IntEnum):
    CLEAN = RmfTaskType.TYPE_CLEAN
    LOOP = RmfTaskType.TYPE_LOOP
    DELIVERY = RmfTaskType.TYPE_DELIVERY


class TaskProgress(BaseModel):
    status: str


class Task(BaseModel):
    task_id: str
    authz_grp: Optional[str]
    summary: TaskSummary
    progress: TaskProgress


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
