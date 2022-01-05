import time


def now() -> int:
    """
    Return current unix time in millis
    """
    return int(time.time() * 1000)
