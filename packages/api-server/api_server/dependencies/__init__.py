from typing import Awaitable, Callable, Dict, Optional, TypeVar

from fastapi import Query
from tortoise.queryset import QuerySet

from ..models import Pagination
from .auth import auth_scheme

ResultT = TypeVar("ResultT")
WithBaseQuery = Callable[[QuerySet], Awaitable[Pagination[ResultT]]]


def base_query_params(field_mapping: Dict[str, str] = None) -> WithBaseQuery[ResultT]:
    """
    :param field_mapping: A dict mapping the order fields to the fields used to build the
        query. e.g. a url of `?order_by=order_field` and a field mapping of `{"order_field": "db_field"}`
        will order the query result according to `db_field`.
    """
    field_mapping = field_mapping or {}

    def dep(
        limit: int = Query(100, le=100),
        offset: int = 0,
        order_by: Optional[str] = Query(
            None,
            description="common separated list of fields to order by, prefix with '-' to sort descendingly.",
        ),
    ) -> WithBaseQuery:
        async def do_query(query: QuerySet):
            count = await query.count()
            query = query.limit(limit).offset(offset)
            if order_by is not None:
                order_fields = []
                order_values = order_by.split(",")
                for v in order_values:
                    if v[0] in ["-", "+"]:
                        stripped = v[1:]
                        order_fields.append(
                            v[0] + field_mapping.get(stripped, stripped)
                        )
                    else:
                        order_fields.append(field_mapping.get(v, v))
                query = query.order_by(*order_fields)
            items = await query
            return Pagination.construct(total_count=count, items=items)

        return do_query

    return dep
