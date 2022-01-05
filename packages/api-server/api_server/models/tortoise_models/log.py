from enum import Enum

from tortoise.fields import CharEnumField, IntField, TextField


class LogEntryTier(Enum):
    uninitialized = "uninitialized"
    info = "info"
    warning = "warning"
    error = "error"


class LogMixin:
    seq = IntField()
    unix_millis_time = IntField(null=False, index=True)
    tier = CharEnumField(LogEntryTier, max_length=255)
    text = TextField()
