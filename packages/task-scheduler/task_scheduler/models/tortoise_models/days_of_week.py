from datetime import datetime, timedelta
from typing import List

from tortoise import fields, models


class DayOfWeekService:
    async def get_days_of_week(self, id) -> List[bool]:
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

    async def get_first_active_day(self, id: int, current_datetime: datetime):
        days = await self.get_days_of_week(id)

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
