from tortoise.models import Model

from .json_mixin import JsonMixin


class IngestorState(Model, JsonMixin):
    pass
