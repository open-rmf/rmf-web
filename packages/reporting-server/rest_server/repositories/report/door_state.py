from datetime import timezone
from typing import Optional

from dateutil import parser
from models.pydantic_models import DoorState_Pydantic
from models.tortoise_models.door_state import DoorState


async def get_door_state(
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

    queryset = (
        DoorState.filter(**query)
        .prefetch_related("door")
        .offset(offset)
        .limit(limit)
        .order_by("-created")
    )

    return await DoorState_Pydantic.from_queryset(queryset)
