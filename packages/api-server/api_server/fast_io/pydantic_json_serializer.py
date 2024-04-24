import pydantic
from socketio.packet import _json as default_json


class PydanticJsonSerializer:
    def loads(self, *args, **kwargs):
        return default_json.loads(*args, **kwargs)

    def dumps(self, obj, *args, **kwargs):
        # Hacky workaround to allow pydantic objects to be serialized.
        # The problem is that the default json serializer cannot serialize some
        # pydantic objects. We also cannot use a custom `JSONEncoder` as the
        # design is flawed, it requires an incompatible object to be converted to an
        # compatible one, so to serialize a pydantic object, it needs to be converted
        # to a dict, letting pydantic do the serialization will cause it to be
        # encoded as a string.
        #
        # In this workaround, we are assuming all payloads which involves pydantic objects
        # is in a 2 item tuple with the room name and the data. We then build the json
        # manually (if the assumption is broken, then it will broken invalid json!).
        if isinstance(obj, list) and isinstance(obj[1], pydantic.BaseModel):
            return f'["{obj[0]}",{obj[1].model_dump_json()}]'
        return default_json.dumps(obj, *args, **kwargs)
