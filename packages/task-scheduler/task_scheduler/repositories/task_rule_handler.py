from typing import List

from task_scheduler.models.pydantic_models import TaskRule_Pydantic
from task_scheduler.models.tortoise_models import DaysOfWeek, TaskRule
from task_scheduler.models.tortoise_models.helpers.task_rule_definition import (
    FrequencyEnum,
)


class TaskRuleRepository:
    @staticmethod
    async def create(payload: dict) -> TaskRule:
        """
        Create a scheduled task.

        :return: scheduled task
        """
        days_of_week = payload.get("days_of_week", None)

        if days_of_week is not None:
            days = payload["days_of_week"]
            days_of_week = await DaysOfWeek.create(
                monday=days[0],
                tuesday=days[1],
                wednesday=days[2],
                thursday=days[3],
                friday=days[4],
                saturday=days[5],
                sunday=days[6],
            )

        if (
            payload.get("frequency_type") != FrequencyEnum.ONCE
            and payload.get("end_datetime") is None
        ):
            raise Exception(
                "If you want to schedule a task more than ONCE you should set an end_datetime"
            )
        return await TaskRule.create(
            name=payload.get("name"),
            task_type=payload.get("task_type"),
            frequency=payload.get("frequency"),
            frequency_type=payload.get("frequency_type"),
            start_datetime=payload.get("start_datetime"),
            end_datetime=payload.get("end_datetime"),
            days_of_week=days_of_week,
            # args=payload.get("args")
        )

    @staticmethod
    async def get(offset: int, limit: int) -> List[TaskRule]:
        """
        Get the list of task rules.

        :return: list of task rules
        """
        query = {}

        return await TaskRule_Pydantic.from_queryset(
            TaskRule.filter(**query).offset(offset).limit(limit).order_by("-created_at")
        )

    @staticmethod
    async def delete(task_rule_id: int) -> None:
        """
        Delete a task rule .

        :return: None
        """
        task_rule = await TaskRule.get(id=task_rule_id)
        if task_rule is None:
            raise Exception("Task rule id not found: %s" % task_rule_id)

        await task_rule.get(id=task_rule_id).delete()
