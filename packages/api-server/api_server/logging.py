import logging
import typing

# from api_server.models import User
from fastapi import Request
from termcolor import colored
from termcolor._types import Color

from api_server.models import User

log_colors: dict[str, Color] = {
    "DEBUG": "grey",
    "INFO": "blue",
    "WARNING": "yellow",
    "ERROR": "red",
    "CRITICAL": "red",
}


class _CustomLogRecord(logging.LogRecord):
    """A Customized LogRecord for typing"""

    request: Request
    user: User | None


class LogfmtFormatter(logging.Formatter):
    """A formatter to formats to logfmt"""

    def fields(self, record: logging.LogRecord):
        # Extract fields from the log record
        msg = record.getMessage().replace('"', '\\"').rstrip()
        fields = {
            "msg": msg,
            "level": record.levelname,
            "ts": self.formatTime(
                record, datefmt=f"%Y-%m-%dT%H:%M:%S.{int(record.msecs)}%z"
            ),
            "src": f"{record.filename}:{record.lineno}",
            "funcName": record.funcName,
        }

        # Add contextual information
        if hasattr(record, "request"):
            record = typing.cast(_CustomLogRecord, record)
            client = record.request.client
            fields["client"] = f"{client.host}:{client.port}" if client else "None"
            fields["user"] = record.user.username if record.user else "None"

        if record.exc_info:
            fields["exc_info"] = self.formatException(record.exc_info)

        return fields

    def format(self, record):
        fields = self.fields(record)
        color = log_colors.get(record.levelname, None)
        logfmt_pairs = [f'{key}="{value}"' for key, value in fields.items()]
        return colored(" ".join(logfmt_pairs), color)


class Logger(logging.LoggerAdapter):
    """A customized `LoggerAdapter` that adds extra contextual info like client and user"""

    def __init__(self, logger: logging.Logger, request: Request, user: User | None):
        super().__init__(logger)
        self.request = request
        self.user = user

    def process(self, msg, kwargs):
        kwargs["extra"] = {"request": self.request, "user": self.user}
        return msg, kwargs


def logger(request: Request) -> Logger:
    l = logging.getLogger()
    # Note that user is always `None` here to avoid circular dependency if we use `user_dep`.
    # The authenticator will fill in the user instead.
    return Logger(l, request, user=None)
