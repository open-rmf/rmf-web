from rmf_lift_msgs.msg import LiftState as RmfLiftState
from rosidl_runtime_py.convert import message_to_ordereddict
from tortoise import fields
from tortoise.models import Model

from .ros_convert import update_message_from_dict


class LiftState(Model):
    lift_name = fields.CharField(255, pk=True)
    data = fields.JSONField()

    @staticmethod
    async def from_rmf(rmf_lift_state: RmfLiftState):
        return LiftState(
            lift_name=rmf_lift_state.lift_name,
            data=message_to_ordereddict(rmf_lift_state),
        )

    def to_rmf(self):
        rmf = RmfLiftState()
        update_message_from_dict(rmf, self.data)
        return rmf
