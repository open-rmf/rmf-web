# from datetime import datetime, timedelta
# from enum import Enum

# from task_scheduler.models.tortoise_models.days_of_week import DaysOfWeek
# from task_scheduler.models.tortoise_models.task_rule import TaskRule
# from task_scheduler.models.tortoise_models.scheduled_task import ScheduledTask
# from .task_rule import is_time_between, calculate_next_time


# class FrequencyEnum(str, Enum):
#     MONTHLY = "Monthly"
#     WEEKLY = "Weekly"
#     DAILY = "Daily"
#     HOURLY = "Hourly"
#     MINUTELY = "Minutely"
#     ONCE = "Once"
#     CUSTOM = "Custom"


# class SimpleScheduleManager:
#     @staticmethod
#     async def handleScheduleWithOneDate(instance):
#         last_task_datetime = instance.start_datetime

#         if instance.frequency_type == FrequencyEnum.ONCE:
#             await ScheduledTask.create(
#                 task_type=instance.task_type,
#                 # Need to modify this
#                 task_datetime=instance.first_day_to_apply_rule,
#                 rule=instance,
#             )
#             return

#         time_delta_content = TaskRule.service.get_timedelta(
#             instance.frequency_type, instance.frequency, instance.start_datetime
#         )

#         while is_time_between(
#             instance.start_datetime, instance.end_datetime, last_task_datetime
#         ):
#             await ScheduledTask.create(
#                 task_type=instance.task_type,
#                 # Need to modify this
#                 task_datetime=last_task_datetime,
#                 rule=instance,
#             )
#             last_task_datetime = calculate_next_time(
#                 last_task_datetime, time_delta_content)


# class MultipleDaysScheduleManager:
#     @staticmethod
#     async def handleScheduleWithMultipleDays(instance):
#         last_task_datetime = instance.start_datetime

#         if instance.frequency_type == FrequencyEnum.ONCE:
#             await ScheduledTask.create(
#                 task_type=instance.task_type,
#                 # Need to modify this
#                 task_datetime=instance.first_day_to_apply_rule,
#                 rule=instance,
#             )
#             return

#         time_delta_content = TaskRule.service.get_timedelta(
#             instance.frequency_type, instance.frequency, instance.start_datetime
#         )

#         while is_time_between(
#             instance.start_datetime, instance.end_datetime, last_task_datetime
#         ):
#             await ScheduledTask.create(
#                 task_type=instance.task_type,
#                 # Need to modify this
#                 task_datetime=last_task_datetime,
#                 rule=instance,
#             )
#             last_task_datetime = calculate_next_time(
#                 last_task_datetime, time_delta_content)
