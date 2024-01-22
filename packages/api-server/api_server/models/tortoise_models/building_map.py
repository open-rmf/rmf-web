from typing import Any, cast

from tortoise.fields import CharField, Field, JSONField
from tortoise.models import Model


class BuildingMap(Model):
    id_ = CharField(255, pk=True, source_field="id")
    data = cast(Field[dict[str, Any]], JSONField())
