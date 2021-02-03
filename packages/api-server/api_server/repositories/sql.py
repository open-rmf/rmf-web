import logging

from rmf_door_msgs.msg import DoorState as RmfDoorState

from ..models import DoorState


class SqlRepository():
    '''
    NOTE: tortoise-orm must already be inited before using any of the methods in this class.
    '''

    def __init__(self, logger: logging.Logger = None):
        self.logger = logger or logging.getLogger(self.__class__.__name__)

    async def write_door_state(self, rmf_door_state: RmfDoorState):
        door_state = DoorState.from_rmf(rmf_door_state)
        await door_state.save()
        self.logger.debug(f'written door_state ("{door_state.door_name}") to database')
