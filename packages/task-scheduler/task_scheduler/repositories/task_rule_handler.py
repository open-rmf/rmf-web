from models.tortoise import TaskRule


class TaskRuleManager:
    @staticmethod
    async def get_rules():
        """
        Get the list of scheduled tasks.

        :return: list of scheduled tasks
        """
        return TaskRule.all()

    @staticmethod
    async def create_rule():
        """
        Create a scheduled task.

        :return: scheduled task
        """

        return TaskRule.create()

    # description = fields.TextField()
    # task_type: TaskTypeEnum = fields.CharEnumField(TaskTypeEnum)
    # frequency = fields.IntField()
    # frequency_type: FrequencyEnum = fields.CharEnumField(
    #     FrequencyEnum, default=FrequencyEnum.MINUTELY
    # )
    # time_of_day = fields.TimeField()
    # start_date = fields.DateTimeField()
    # end_date = fields.DateTimeField()
    # args = fields.JSONField()

    @staticmethod
    async def delete_rule():
        """
        Delete a scheduled task.

        :return: scheduled task
        """
        pass
        # return TaskRule.delete()
