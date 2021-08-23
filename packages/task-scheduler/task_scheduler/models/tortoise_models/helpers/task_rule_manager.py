from datetime import datetime, timedelta
from typing import List

from task_scheduler.models.tortoise_models.days_of_week import DaysOfWeek
from task_scheduler.models.tortoise_models.scheduled_task import ScheduledTask

from .datetime_manager import DatetimeManager
from .task_rule_definition import FrequencyEnum


class SimpleScheduleManager:
    async def schedule_tasks(self):
        self.task_rule_instance.first_day_to_apply_rule = (
            self.task_rule_instance.start_datetime
        )
        last_task_datetime = self.task_rule_instance.start_datetime

        if self.task_rule_instance.frequency_type == FrequencyEnum.ONCE:
            await ScheduledTask.create(
                task_type=self.task_rule_instance.task_type,
                # Need to modify this
                task_datetime=self.task_rule_instance.first_day_to_apply_rule,
                rule=self.task_rule_instance,
            )
            return

        time_delta_content = DatetimeManager.get_timedelta(
            self.task_rule_instance.frequency_type,
            self.task_rule_instance.frequency,
            self.task_rule_instance.start_datetime,
        )

        while DatetimeManager.is_time_between(
            self.task_rule_instance.start_datetime,
            self.task_rule_instance.end_datetime,
            last_task_datetime,
        ):
            await ScheduledTask.create(
                task_type=self.task_rule_instance.task_type,
                task_datetime=last_task_datetime,
                rule=self.task_rule_instance,
            )

            last_task_datetime = DatetimeManager.calculate_next_time(
                last_task_datetime, time_delta_content
            )

    def __init__(self, task_rule_instance):
        self.task_rule_instance = task_rule_instance


class MultipleDaysScheduleManager:
    def is_invalid_frequency(self):
        return (
            self.task_rule_instance.frequency_type == FrequencyEnum.DAILY
            or self.task_rule_instance.frequency_type == FrequencyEnum.HOURLY
            or self.task_rule_instance.frequency_type == FrequencyEnum.MINUTELY
        )

    async def schedule_tasks(self):
        week_id = self.task_rule_instance.days_of_week.id
        first_active_day = await DaysOfWeek.service.get_first_active_day(
            week_id=week_id,
            current_datetime=self.task_rule_instance.start_datetime,
        )
        self.task_rule_instance.first_day_to_apply_rule = first_active_day
        # We pick first_day_to_apply_rule because the task can start with
        # earlier but not the execution
        # last_task_datetime = task_rule_instance.first_day_to_apply_rule
        # [0,6]
        active_weekdays = await DaysOfWeek.service.get_active_days_of_week(week_id)

        if self.is_invalid_frequency():
            raise Exception(
                "You cannot set the rule DAILY|HOURLY|MINUTELY and pick weekdays"
            )

        if self.task_rule_instance.frequency_type == FrequencyEnum.ONCE:

            week_day_tasks = MultipleDaysScheduleManager._get_initialized_datetime_list(
                active_weekdays, self.task_rule_instance.first_day_to_apply_rule
            )
            for weekday in week_day_tasks:
                if weekday is None:
                    continue

                await ScheduledTask.create(
                    task_type=self.task_rule_instance.task_type,
                    task_datetime=weekday,
                    rule=self.task_rule_instance,
                )
            return

        week_day_tasks = MultipleDaysScheduleManager._get_initialized_datetime_list(
            active_weekdays, self.task_rule_instance.first_day_to_apply_rule
        )

        while DatetimeManager.is_time_between(
            self.task_rule_instance.start_datetime,
            self.task_rule_instance.end_datetime,
            week_day_tasks[active_weekdays[0]],
        ):
            # for active_weekday in active_weekdays:
            for index, weekday_datetime in enumerate(week_day_tasks):
                if weekday_datetime is None:
                    continue

                if self.task_rule_instance.end_datetime < weekday_datetime:
                    break

                await ScheduledTask.create(
                    task_type=self.task_rule_instance.task_type,
                    task_datetime=weekday_datetime,
                    rule=self.task_rule_instance,
                )
                time_delta_content = DatetimeManager.get_timedelta(
                    self.task_rule_instance.frequency_type,
                    self.task_rule_instance.frequency,
                    self.task_rule_instance.start_datetime,
                )
                # Add the frecuency again
                week_day_tasks[index] = DatetimeManager.calculate_next_time(
                    weekday_datetime, time_delta_content
                )

    # Calculate date distance between weekdays

    @staticmethod
    def get_weekday_distance(weekday_1: int, weekday_2: int) -> int:
        return abs(weekday_1 - weekday_2)

    # Get first day of next week
    @staticmethod
    def get_monday_of_next_week(current_day: datetime) -> datetime:
        return current_day + timedelta(days=7 - current_day.weekday())

    @staticmethod
    def _get_initialized_datetime_list(
        active_weekdays: List[int], last_task_datetime
    ) -> List[datetime]:
        week_day_tasks: List[datetime] = [None, None, None, None, None, None, None]
        count = 0
        # Initialize week with first values
        for weekday in active_weekdays:
            if count == 0:
                week_day_tasks[weekday] = last_task_datetime

            weekday_distance = MultipleDaysScheduleManager.get_weekday_distance(
                weekday, last_task_datetime.weekday()
            )

            week_day_tasks[weekday] = DatetimeManager.calculate_next_time(
                last_task_datetime, timedelta(days=weekday_distance)
            )
        return week_day_tasks

    def __init__(self, task_rule_instance):
        self.task_rule_instance = task_rule_instance


class SchedulerFactory:
    @staticmethod
    def get_schedule_manager(task_rule_instance) -> object:

        if task_rule_instance.days_of_week is None:
            return SimpleScheduleManager(task_rule_instance)

        return MultipleDaysScheduleManager(task_rule_instance)
