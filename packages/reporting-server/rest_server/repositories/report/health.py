from datetime import datetime, timezone
from typing import List, Optional

from dateutil import parser, tz
from models.health import HealthStatus, HealthStatus_Pydantic


async def get_health(
    offset: int,
    limit: int,
    to_log_date: Optional[str] = None,
    from_log_date: Optional[str] = None,
):
    query = {}

    if from_log_date:
        local_time = parser.parse(from_log_date)
        utc_time = local_time.astimezone(timezone.utc)
        query["created__gte"] = utc_time

    if to_log_date:
        to_log_local_time = parser.parse(to_log_date)
        to_log_utc_time = to_log_local_time.astimezone(timezone.utc)
        query["created__lt"] = to_log_utc_time

    return await HealthStatus_Pydantic.from_queryset(
        HealthStatus.filter(**query).offset(offset).limit(limit)
    )
