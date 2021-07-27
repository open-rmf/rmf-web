from tortoise import Model

from api_server.models.tortoise_models.json_mixin import JsonMixin


class IngestorState(Model, JsonMixin):
    pass
