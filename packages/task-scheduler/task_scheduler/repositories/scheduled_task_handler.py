from models.tortoise import ScheduledTask


class ScheduledTaskManager:
    @staticmethod
    async def get_scheduled_tasks():
        """
        Get the list of scheduled tasks.

        :return: list of scheduled tasks
        """
        return ScheduledTask.all()

    @staticmethod
    async def create_scheduled_task():
        """
        Create a scheduled task.

        :return: scheduled task
        """
        pass
        # return ScheduledTask.create()

    @staticmethod
    async def delete_scheduled_task():
        """
        Delete a scheduled task.

        :return: scheduled task
        """
        pass
        # return ScheduledTask.delete()
