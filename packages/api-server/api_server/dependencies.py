from typing import Optional

from fastapi import Query

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
