from enum import Enum


class FrequencyEnum(str, Enum):
    MONTHLY = "Monthly"
    WEEKLY = "Weekly"
    DAILY = "Daily"
    HOURLY = "Hourly"
    MINUTELY = "Minutely"
    ONCE = "Once"
    CUSTOM = "Custom"


class WeekDayEnum(Enum):
    MONDAY = 0
    TUESDAY = 1
    WEDNESDAY = 2
    THURSDAY = 3
    FRIDAY = 4
    SATURDAY = 5
    SUNDAY = 6


MONTH_DAYS = [0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
MONTH_DAYS_LEAP = [0, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
