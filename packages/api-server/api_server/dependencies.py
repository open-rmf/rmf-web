from typing import Any, Callable, Coroutine, Optional, Union

from fastapi import Depends, Query

from api_server.base_app import BaseApp
from api_server.repositories import TaskRepository

from .fast_io import SubscriptionRequest
from .models import Pagination, User
from .repositories import RmfRepository


def pagination_query(
    limit: int = Query(100, gt=0, le=100),
    offset: int = Query(0, ge=0, le=1000000),
    order_by: Optional[str] = Query(
        None,
        description="common separated list of fields to order by, prefix with '-' to sort descendingly.",
    ),
) -> Pagination:
    return Pagination(limit=limit, offset=offset, order_by=order_by)


def rmf_repo(
    user_dep: Callable[..., Union[Coroutine[Any, Any, User], User]]
) -> Callable[..., RmfRepository]:
    def dep(user: User = Depends(user_dep)):
        return RmfRepository(user)

    return dep


def task_repo_dep(app: BaseApp) -> Callable[..., TaskRepository]:
    def dep(user: User = Depends(app.user_dep)):
        return TaskRepository(user)

    return dep


# hacky way to get the sio user
def sio_user(req: SubscriptionRequest) -> User:
    return req.session["user"]
