from rmf_door_msgs.msg import DoorState as RmfDoorState

from ..models import DoorState


class SqlRepository():
    '''
    NOTE: tortoise-orm must already be inited before using any of the methods in this class.
    '''

    async def write_door_state(rmf_door_state: RmfDoorState):
        door_state = DoorState.from_rmf(rmf_door_state)
        await door_state.save()
