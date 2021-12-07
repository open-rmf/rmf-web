from datetime import datetime

from .rmf_api.task_state import TaskState as BaseTaskState
from .tortoise_models import TaskState as DbTaskState


class TaskState(BaseTaskState):
    @staticmethod
    def from_db(task_state: DbTaskState) -> "TaskState":
        return TaskState(**task_state.data)

    async def save(self) -> None:
        await DbTaskState.update_or_create(
            {
                "data": self.json(),
                "category": self.category,
                "unix_millis_start_time": self.unix_millis_start_time
                and datetime.fromtimestamp(self.unix_millis_start_time / 1000),
                "unix_millis_finish_time": self.unix_millis_finish_time
                and datetime.fromtimestamp(self.unix_millis_finish_time / 1000),
            },
            id_=self.booking.id,
        )
