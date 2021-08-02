from typing import Callable, Dict, Optional, TypeVar

from fastapi import Depends, Query
from tortoise.queryset import QuerySet

from api_server.base_app import BaseApp
from api_server.models import Pagination, User
from api_server.repositories.rmf import RmfRepository


def pagination_query(
    limit: int = Query(100, gt=0, le=100),
    offset: int = Query(0, ge=0, le=1000000),
    order_by: Optional[str] = Query(
        None,
        description="common separated list of fields to order by, prefix with '-' to sort descendingly.",
    ),
) -> Pagination:
    return Pagination(limit=limit, offset=offset, order_by=order_by)


def rmf_repo(user_dep: Callable[..., User]) -> RmfRepository:
    def dep(user: User = Depends(user_dep)):
        return RmfRepository(user)

    return dep
