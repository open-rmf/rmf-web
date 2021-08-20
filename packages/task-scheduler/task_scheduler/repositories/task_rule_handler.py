from typing import Any, List

from task_scheduler.models.tortoise_models import TaskRule


class TaskRuleRepository:
    @staticmethod
    async def create(payload: Any) -> TaskRule:
        """
        Create a scheduled task.

        :return: scheduled task
        """
        return TaskRule.create(payload)

    # description = fields.TextField()
    # task_type: TaskTypeEnum = fields.CharEnumField(TaskTypeEnum)
    # frequency = fields.IntField()
    # frequency_type: FrequencyEnum = fields.CharEnumField(
    #     FrequencyEnum, default=FrequencyEnum.MINUTELY
    # )
    # first_day_to_apply_rule = fields.TimeField()
    # start_date = fields.DateTimeField()
    # end_date = fields.DateTimeField()
    # args = fields.JSONField()

    @staticmethod
    async def get() -> List[TaskRule]:
        """
        Get the list of scheduled tasks.

        :return: list of scheduled tasks
        """
        return TaskRule.all()

    @staticmethod
    async def delete(id: int) -> None:
        """
        Delete a scheduled task.

        :return: None
        """
        pass
        return TaskRule.delete(id)
