import tortoise.functions as tfuncs
from tortoise.expressions import Q
from tortoise.queryset import MODEL, QuerySet

from api_server.models.pagination import Pagination


def add_pagination(
    query: QuerySet[MODEL],
    pagination: Pagination,
    field_mappings: dict[str, str] | None = None,
    group_by: str | None = None,
) -> QuerySet[MODEL]:
    """
    Adds pagination and ordering to a query. If the order field starts with `label=`, it is
    assumed to be a label and label sorting will used. In this case, the model must have
    a reverse relation named "labels" and the `group_by` param is required.

    :param field_mapping: A dict mapping the order fields to the fields used to build the
        query. e.g. a url of `?order_by=order_field` and a field mapping of `{"order_field": "db_field"}`
        will order the query result according to `db_field`.
    :param group_by: Required when sorting by labels, must be the foreign key column of the label table.
    """
    field_mappings = field_mappings or {}
    annotations = {}
    query = query.limit(pagination.limit).offset(pagination.offset)
    if pagination.order_by is not None:
        order_fields = []
        order_values = pagination.order_by.split(",")
        for v in order_values:
            # perform the mapping after stripping the order prefix
            order_prefix = ""
            order_field = v
            if v[0] in ["-", "+"]:
                order_prefix = v[0]
                order_field = v[1:]
            order_field = field_mappings.get(order_field, order_field)

            # add annotations required for sorting by labels
            if order_field.startswith("label="):
                f = order_field[6:]
                annotations[f"label_sort_{f}"] = tfuncs.Max(
                    "labels__label_value",
                    _filter=Q(labels__label_name=f),
                )
                order_field = f"label_sort_{f}"

            order_fields.append(order_prefix + order_field)

        query = query.annotate(**annotations)
        if group_by is not None:
            query = query.group_by(group_by)
        query = query.order_by(*order_fields)
    return query
