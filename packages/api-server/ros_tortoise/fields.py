from datetime import datetime
from typing import Any

from tortoise import fields
from builtin_interfaces.msg import Time as RosTime
from .ros_time import ros_to_py_datetime, py_to_ros_time


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
