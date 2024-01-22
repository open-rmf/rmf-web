from typing import Any, cast

from tortoise.fields import CharField, Field, JSONField


class JsonMixin:
    id_ = CharField(255, pk=True, source_field="id")
    data = cast(Field[dict[str, Any]], JSONField())
