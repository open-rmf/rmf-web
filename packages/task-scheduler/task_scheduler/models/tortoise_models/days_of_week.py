from datetime import datetime, timedelta
from typing import List

from tortoise import fields, models


class DayOfWeekService:
    async def get_first_active_day(self, id: int, current_datetime: datetime):
        days = await self._get_days_of_week(id)

        current_day: int = current_datetime.weekday()
        days_count = 0
        # rest of this week
        for day in days[current_day:]:
            if day == True:
                return current_datetime + timedelta(days=days_count)
            days_count += 1

        for day in days[:current_day]:
            if day == True:
                return current_datetime + timedelta(days=days_count)
                # If next week
            days_count += 1

    # return the boolean True positions of a list

    async def get_active_days_of_week(self, id: int) -> List[int]:
        days = await self._get_days_of_week(id)
        return self._get_active_days_of_week(days)

    def _get_active_days_of_week(self, list_of_days: List[bool]) -> List[int]:
        return [i for i, x in enumerate(list_of_days) if x]

    async def _get_days_of_week(self, id: int) -> List[bool]:
        available_days = await DaysOfWeek.filter(id=id).values(
            "monday",
            "tuesday",
            "wednesday",
            "thursday",
            "friday",
            "saturday",
            "sunday",
        )
        print(available_days[0])
        return [value for key, value in available_days[0].items()]


class DaysOfWeek(models.Model):
    id = fields.IntField(pk=True)
    sunday = fields.BooleanField(default=False)
    monday = fields.BooleanField(default=False)
    tuesday = fields.BooleanField(default=False)
    wednesday = fields.BooleanField(default=False)
    thursday = fields.BooleanField(default=False)
    friday = fields.BooleanField(default=False)
    saturday = fields.BooleanField(default=False)

    service = DayOfWeekService()
