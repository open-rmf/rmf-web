from typing import Any, Optional

from tortoise import fields

from datetime import datetime, timezone
from builtin_interfaces.msg import Time as RosTime


def ros_to_py_datetime(ros_time: RosTime) -> datetime:
    '''
    ros_time is assumed to be utc.
    '''
    return datetime.fromtimestamp(ros_time.sec + ros_time.nanosec / 1000000000, timezone.utc)


def py_to_ros_time(py_datetime: datetime) -> RosTime:
    '''
    ros_time is assumed to be utc.
    '''
    utc_timestamp = py_datetime.timestamp() - py_datetime.utcoffset().total_seconds()
    return RosTime(
        sec=int(utc_timestamp),
        nanosec=int((utc_timestamp % 1) * 1000000000)
    )

class RosTimeField(fields.DatetimeField):
    '''
    Serializes `builtin_interfaces/Time` to DateTime in DB.
    '''

    def to_db_value(self, ros_time: RosTime, instance):
        py_datetime = ros_to_py_datetime(ros_time)
        return super().to_db_value(py_datetime, instance)

    def to_python_value(self, db_value: Any) -> Optional[RosTime]:
        py_datetime = super().to_python_value(db_value)
        if py_datetime is None:
            return None
        return py_to_ros_time(py_datetime)
