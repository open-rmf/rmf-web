from datetime import datetime
from typing import Tuple

from fastapi import Depends, Query

from api_server import clock

from .fast_io import SubscriptionRequest
from .models import Pagination, User


def pagination_query(
    limit: int | None = Query(None, gt=0, le=1000, description="defaults to 100"),
    offset: int | None = Query(None, ge=0, description="defaults to 0"),
    order_by: str
    | None = Query(
        None,
        description="common separated list of fields to order by, prefix with '-' to sort descendingly.",
    ),
) -> Pagination:
    limit = limit or 100
    offset = offset or 0
    return Pagination(
        limit=limit,
        offset=offset,
        order_by=order_by.split(",") if order_by else [],
    )


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


def start_time_between_query(
    start_time_between: str = Query(
        None,
        description="""
        The period of starting time to fetch, in unix millis.

        This must be a comma separated string, 'X,Y' to fetch between X millis and Y millis inclusive.

        Example:
            "1000,2000" - Fetches logs between unix millis 1000 and 2000.
        """,
    ),
    now: int = Depends(clock.now),
) -> Tuple[datetime, datetime] | None:
    if start_time_between is None:
        return None
    if start_time_between.startswith("-"):
        # Cap at 0 millis
        period = (
            datetime.fromtimestamp(0),
            datetime.fromtimestamp(now / 1000),
        )
    else:
        parts = start_time_between.split(",")
        period = (
            datetime.fromtimestamp(int(parts[0]) / 1000),
            datetime.fromtimestamp(int(parts[1]) / 1000),
        )
    return period


def finish_time_between_query(
    finish_time_between: str = Query(
        None,
        description="""
        The period of finishing time to fetch, in unix millis.

        This must be a comma separated string, 'X,Y' to fetch between X millis and Y millis inclusive.

        Example:
            "1000,2000" - Fetches logs between unix millis 1000 and 2000.
            "-60000" - Fetches logs in the last minute.
        """,
    ),
    now: int = Depends(clock.now),
) -> Tuple[datetime, datetime] | None:
    if finish_time_between is None:
        return None
    if finish_time_between.startswith("-"):
        # Cap at 0 millis
        period = (
            datetime.fromtimestamp(0),
            datetime.fromtimestamp(now / 1000),
        )
    else:
        parts = finish_time_between.split(",")
        period = (
            datetime.fromtimestamp(int(parts[0]) / 1000),
            datetime.fromtimestamp(int(parts[1]) / 1000),
        )
    return period
