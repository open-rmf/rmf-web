from typing import Dict, Optional

from tortoise.queryset import MODEL, QuerySet

from api_server.models.pagination import Pagination


def add_pagination(
    query: QuerySet[MODEL],
    pagination: Pagination,
    field_mappings: Optional[Dict[str, str]] = None,
) -> QuerySet[MODEL]:
    """
    Adds pagination and ordering to a query.

    :param field_mapping: A dict mapping the order fields to the fields used to build the
        query. e.g. a url of `?order_by=order_field` and a field mapping of `{"order_field": "db_field"}`
        will order the query result according to `db_field`.
    """
    field_mappings = field_mappings or {}
    query = query.limit(pagination.limit).offset(pagination.offset)
    if pagination.order_by is not None:
        order_fields = []
        order_values = pagination.order_by.split(",")
        for v in order_values:
            if v[0] in ["-", "+"]:
                stripped = v[1:]
                order_fields.append(v[0] + field_mappings.get(stripped, stripped))
            else:
                order_fields.append(field_mappings.get(v, v))
        query = query.order_by(*order_fields)
    return query
