from models.raw_log import LogLevel


def get_log_type(fullstring):
    if "critical" in fullstring.lower():
        return LogLevel.CRITICAL
    elif "error" in fullstring.lower():
        return LogLevel.ERROR
    elif "warn" in fullstring.lower():
        return LogLevel.WARN
    elif "info" in fullstring.lower():
        return LogLevel.INFO
    elif "debug" in fullstring.lower():
        return LogLevel.DEBUG
    else:
        return LogLevel.UNKNOWN
