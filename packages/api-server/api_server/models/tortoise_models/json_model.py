from typing import Any, Callable

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from tortoise import Model, fields


def json_model(RmfMessage, key_mapper: Callable[[Any], str]):
    """
    Returns a tortoise model that wraps a RMF message in a JSON field.

    :param RmfMessage: A ROS2 message class
    :param key_mapper: A function that, given an instance of RmfMessage, returns
        a string that can be used to uniquely identify the rmf message. The string
        returned will be used as the primary key.

    :example:

    class MyRmfModel(json_model(DoorState, lambda x: x.door_name)):
        pass
    """

    class JsonModel(Model):
        id_ = fields.CharField(255, pk=True, source_field="id")
        data = fields.JSONField()

        @classmethod
        def update_or_create_from_rmf(cls, rmf_msg: RmfMessage):
            id_ = key_mapper(rmf_msg)
            data = message_to_ordereddict(rmf_msg)
            return cls.update_or_create({"data": data}, id_=id_)

        def update_from_rmf(self, rmf_msg: RmfMessage):
            self.id_ = key_mapper(rmf_msg)
            self.data = message_to_ordereddict(rmf_msg)
            return self

        def to_rmf(self):
            rmf_msg = RmfMessage()
            set_message_fields(rmf_msg, self.data)
            return rmf_msg

    return JsonModel
