from datetime import datetime

from fastapi import Depends, Query

from api_server import ros_time

from .models import Pagination


def pagination_query(
    limit: int | None = Query(None, gt=0, le=1000, description="defaults to 100"),
    offset: int | None = Query(None, ge=0, description="defaults to 0"),
    order_by: str | None = Query(
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
    now: int = Depends(ros_time.now),
) -> tuple[int, int]:
    if between.startswith("-"):
        period = (now - int(between[1:]), now)
    else:
        parts = between.split(",")
        period = (int(parts[0]), int(parts[1]))
    return period


def request_time_between_query(
    request_time_between: str = Query(
        None,
        description="""
        The period of request time to fetch, in unix millis.

        This must be a comma separated string, 'X,Y' to fetch between X millis and Y millis inclusive.

        Example:
            "1000,2000" - Fetches logs between unix millis 1000 and 2000.
        """,
    ),
    now: int = Depends(ros_time.now),
) -> tuple[datetime, datetime] | None:
    if request_time_between is None:
        return None
    if request_time_between.startswith("-"):
        # Cap at 0 millis
        period = (
            datetime.fromtimestamp(0),
            datetime.fromtimestamp(now / 1000),
        )
    else:
        parts = request_time_between.split(",")
        period = (
            datetime.fromtimestamp(int(parts[0]) / 1000),
            datetime.fromtimestamp(int(parts[1]) / 1000),
        )
    return period


def start_time_between_query(
    start_time_between: str = Query(
        None,
        description="""
        The period of start time to fetch, in unix millis.

        This must be a comma separated string, 'X,Y' to fetch between X millis and Y millis inclusive.

        Example:
            "1000,2000" - Fetches logs between unix millis 1000 and 2000.
        """,
    ),
    now: int = Depends(ros_time.now),
) -> tuple[datetime, datetime] | None:
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
        The period of finish time to fetch, in unix millis.

        This must be a comma separated string, 'X,Y' to fetch between X millis and Y millis inclusive.

        Example:
            "1000,2000" - Fetches logs between unix millis 1000 and 2000.
        """,
    ),
    now: int = Depends(ros_time.now),
) -> tuple[datetime, datetime] | None:
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
