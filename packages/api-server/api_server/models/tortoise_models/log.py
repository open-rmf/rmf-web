from tortoise.fields import BigIntField, CharEnumField, IntField, TextField

from api_server.models.rmf_api import log_entry


class LogMixin:
    seq = IntField()
    unix_millis_time = BigIntField(null=False, index=True)
    tier = CharEnumField(log_entry.Tier, max_length=255)
    text = TextField()
