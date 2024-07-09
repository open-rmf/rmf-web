from datetime import datetime, timezone

from builtin_interfaces.msg import Time as RosTime

from .ros import get_ros_node


def now() -> int:
    """
    Return current unix time in millis
    """
    ros_time = get_ros_node().get_clock().now()  # pylint: disable=no-member
    return ros_time.nanoseconds // 1000000


def ros_to_py_datetime(ros_time: RosTime) -> datetime:
    """
    The resulting datetime instance is utc.
    """
    return datetime.fromtimestamp(
        ros_time.sec + ros_time.nanosec / 1000000000, timezone.utc
    )


def py_to_ros_time(py_datetime: datetime) -> RosTime:
    """
    ros_time is assumed to be utc.
    """
    utc_timestamp = py_datetime.timestamp()
    return RosTime(
        sec=int(utc_timestamp), nanosec=int((utc_timestamp % 1) * 1000000000)
    )


def convert_to_rmf_time(timestamp: int, ros_now: RosTime) -> RosTime:
    """
    Given a timestamp (in seconds), convert it to rmf time. If rmf is not using simulation time,
    this simply converts to ros time format.
    If it is using sim time, this returns rmf time, relative to the
    difference between the given time and the system's current time.

    For example:
    Given a time of "{ sec: 2500, nanosec: 0 }" (in ros time format).
    Assume that the current system's time is "{ sec: 1500, nanosec: 0 }"
    and that rmf's time is "{ sec: 500, nanosec: 0 }"
    the difference between the system's time and the given time is "{ sec: 1000, nanosec: 0 }"
    and so the result would be given_time - system_time + rmf_time = "{ sec: 1500, nanosec: 0 }"
    """
    ros_time = RosTime(
        sec=timestamp,
        nanosec=0,
    )
    utc_now = py_to_ros_time(datetime.now())
    sec = ros_time.sec - utc_now.sec + ros_now.sec
    nanosec = ros_time.nanosec - utc_now.nanosec + ros_now.nanosec
    if nanosec < 0:
        sec = sec - 1
        nanosec = 1000000000 + nanosec
    return RosTime(
        sec=sec,
        nanosec=nanosec,
    )
