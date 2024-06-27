from tortoise.queryset import MODEL, QuerySet

from api_server.models.pagination import Pagination


def add_pagination(
    query: QuerySet[MODEL],
    pagination: Pagination,
) -> QuerySet[MODEL]:
    """Adds pagination and ordering to a query"""
    return (
        query.limit(pagination.limit)
        .offset(pagination.offset)
        .order_by(*pagination.order_by)
    )
