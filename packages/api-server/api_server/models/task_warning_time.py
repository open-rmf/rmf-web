# pyright: reportGeneralTypeIssues=false

from datetime import datetime

from pydantic import BaseModel

from . import tortoise_models as ttm


class TaskWarningTime(BaseModel):
    id: str
    unix_millis_warn_time: datetime

    @staticmethod
    async def from_db(db_task_warning_time: ttm.TaskWarningTime) -> "TaskWarningTime":
        """
        Convert the db TaskWarningTime model to this pydantic model.
        """
        return TaskWarningTime(
            id=db_task_warning_time.id_,
            unix_millis_warn_time=db_task_warning_time.unix_millis_warn_time,
        )
