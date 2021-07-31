# calculate the next time of a task
#
# @param task_id the task id
# @param task_period the period of the task
# @param task_start_time the start time of the task
# @param task_time_of_day the time of day of the task
# @param task_day_of_week the day of week of the task
# @param task_day_of_month the day of month of the task
# @param task_month_of_year the month of year of the task
# @return the next time of the task
from datetime import datetime, time, timedelta


def is_time_between(start_time, end_time, current_time) -> bool:
    return start_time <= current_time <= end_time


def calculate_next_time(current_time, delta) -> datetime:
    return current_time + delta
    # if delta_type == 'seconds':
    #     return current_time + timedelta(minutes=delta)
    # elif delta_type == 'minutes':
    #     return current_time + timedelta(hours=delta)
    # elif delta_type == 'hours':
    #     return current_time + timedelta(days=delta)
    # elif delta_type == 'days':
    #     return current_time + timedelta(months=delta)
    # elif delta_type == 'month':
    #     return current_time + timedelta(=delta)
    # else:
    #     return None


# def calculate_next_time(task_id, task_period, task_start_time, task_time_of_day, task_day_of_week, task_day_of_month, task_month_of_year):
#     # calculate the next time of the task
#     #
#     # @param task_period the period of the task
#     # @param task_start_time the start time of the task
#     # @param task_time_of_day the time of day of the task
#     # @param task_day_of_week the day of week of the task
#     # @param task_day_of_month the day of month of the task
#     # @param task_month_of_year the month of year of the task
#     # @return the next time of the task
#     def calculate_next_time(task_period, task_start_time, task_time_of_day, task_day_of_week, task_day_of_month, task_month_of_year):
#         # get the current time
#         current_time = time.time()
#         # get the current time of day
#         current_time_of_day = time.localtime(current_time).tm_hour * 60 * 60 + time.localtime(
#             current_time).tm_min * 60 + time.localtime(current_time).tm_sec
#         # get the current day of week
#         current_day_of_week = time.localtime(current_time).tm_wday
#         # get the current day of month
#         current_day_of_month = time.localtime(current_time).tm_mday
#         # get the current month of year
#         current_month_of_year = time.localtime(current_time).tm_mon
#         # get the current time of the task
#         task_time_of_day = int(task_time_of_day)
#         task_day_of_week = int(task_day_of_week)
#         task_day_of_month = int(task_day_of_month)
#         task_month_of_year = int(task_month_of_year)
#         # get the current time of the task
#         current_time_of_task = task_time_of_day * 60 * 60 + task_day_of_week

#         # get the next time of the task
