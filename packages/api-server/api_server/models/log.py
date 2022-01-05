from .rmf_api.log_entry import LogEntry as BaseLogEntry
from .tortoise_models import LogMixin


class LogEntry(BaseLogEntry):
    @staticmethod
    def from_tortoise(tortoise_log: LogMixin):
        return LogEntry.construct(
            seq=tortoise_log.seq,
            unix_millis_time=tortoise_log.unix_millis_time,
            tier=tortoise_log.tier.name,
            text=tortoise_log.text,
        )
