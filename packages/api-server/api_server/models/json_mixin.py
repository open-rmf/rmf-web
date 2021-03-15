from typing import Any, Callable

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from tortoise import fields


def json_mixin(RmfMessage, key_mapper: Callable[[Any], str]):
    """
    Returns a tortoise mixin that wraps a RMF message in a JSON field.

    :param RmfMessage: A ROS2 message class
    :param key_mapper: A function that, given an instance of RmfMessage, returns
        a string that can be used to uniquely identify the rmf message. The string
        returned will be used as the primary key.

    :example:

    class MyRmfModel(Model, json_mixin(DoorState, lambda x: x.door_name)):
        pass
    """

    class JsonMixin:
        id_ = fields.CharField(255, pk=True, source_field="id")
        data = fields.JSONField()

        def update_from_rmf(self, rmf_msg):
            self.id_ = key_mapper(rmf_msg)
            self.data = message_to_ordereddict(rmf_msg)
            return self

        def to_rmf(self):
            rmf_msg = RmfMessage()
            set_message_fields(rmf_msg, self.data)
            return rmf_msg

    return JsonMixin
