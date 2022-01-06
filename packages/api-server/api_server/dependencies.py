from typing import Optional, Tuple

from fastapi import Depends, Query

from api_server import clock

from .fast_io import SubscriptionRequest
from .models import Pagination, User


def pagination_query(
    limit: int = Query(100, gt=0, le=100),
    offset: int = Query(0, ge=0, le=1000000),
    order_by: Optional[str] = Query(
        None,
        description="common separated list of fields to order by, prefix with '-' to sort descendingly.",
    ),
) -> Pagination:
    return Pagination(limit=limit, offset=offset, order_by=order_by)


# hacky way to get the sio user
def sio_user(req: SubscriptionRequest) -> User:
    return req.session["user"]


def between_query(
    between: str = Query(
        "-60000",
        description="""
        The period of time to fetch, in unix millis.

        This can be either a comma separated string or a string prefixed with '-' to fetch the last X millis.

        Example:
            "1000,2000" - Fetches logs between unix millis 1000 and 2000.
            "-60000" - Fetches logs in the last minute.
        """,
    ),
    now: int = Depends(clock.now),
) -> Tuple[int, int]:
    if between.startswith("-"):
        period = (now - int(between[1:]), now)
    else:
        parts = between.split(",")
        period = (int(parts[0]), int(parts[1]))
    return period
