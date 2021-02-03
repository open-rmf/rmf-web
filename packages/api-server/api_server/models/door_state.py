from enum import Enum

from tortoise import fields
from tortoise.models import Model

from rmf_door_msgs.msg import DoorState as RmfDoorState

from .mixins.rmf_door_msgs_mixins import DoorStateMixin
from .door_mode import DoorModeEnum
from ..ros_time import py_to_ros_time, ros_to_py_datetime


class DoorState(DoorStateMixin, Model):
    door_name = fields.CharField(255)  # TODO: foreign key with Door
    current_mode = fields.IntEnumField(DoorModeEnum)

    class Meta:
        unique_together = (("door_name", "door_time"))

    @staticmethod
    def from_rmf(rmf_door_state: RmfDoorState):
        return DoorState(
            door_name=rmf_door_state.door_name,
            current_mode=rmf_door_state.current_mode.value,
            door_time=ros_to_py_datetime(rmf_door_state.door_time),
        )

    async def to_rmf(self):
        await self.fetch_related('door_name', 'current_mode', 'door_time')
        return RmfDoorState(
            door_name=await self.door_name,
            current_mode=await self.current_mode.value,
            door_time=py_to_ros_time(await self.door_time),
        )
