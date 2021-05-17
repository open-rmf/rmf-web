from typing import Callable, Dict, Optional

from fastapi import Query
from tortoise.queryset import QuerySet

from .auth import auth_scheme

WithBaseQuery = Callable[[QuerySet], QuerySet]


def base_query_params(field_mapping: Dict[str, str] = None):
    field_mapping = field_mapping or {}

    def dep(
        offset: int = 0,
        order_by: Optional[str] = Query(
            None,
            description="common separated list of fields to order by, prefix with '-' to sort descendingly.",
        ),
    ) -> WithBaseQuery:
        def build_query(query: QuerySet):
            query = query.limit(100).offset(offset)
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
            return query

        return build_query

    return dep
