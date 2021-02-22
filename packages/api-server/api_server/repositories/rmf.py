from abc import ABC, abstractmethod
from typing import Dict

from building_map_msgs.msg import Door
from rmf_door_msgs.msg import DoorState


class RmfRepository(ABC):
    @abstractmethod
    async def update_door_state(self, door_state: DoorState) -> None:
        pass

    @abstractmethod
    async def read_door_states(self) -> Dict[str, DoorState]:
        pass

    @abstractmethod
    async def update_door(self, door: Door) -> None:
        pass
