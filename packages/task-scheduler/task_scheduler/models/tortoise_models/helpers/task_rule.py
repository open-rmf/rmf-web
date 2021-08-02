from datetime import datetime


def is_time_between(start_time, end_time, current_time) -> bool:
    return start_time <= current_time <= end_time


def calculate_next_time(current_time, delta) -> datetime:
    return current_time + delta
