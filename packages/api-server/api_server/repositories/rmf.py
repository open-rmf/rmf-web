from abc import ABC, abstractmethod
from typing import Dict, Optional, Sequence

from building_map_msgs.msg import Door
from rmf_dispenser_msgs.msg import DispenserState
from rmf_door_msgs.msg import DoorState
from rmf_lift_msgs.msg import LiftState

from ..models import DoorHealth, LiftHealth


class RmfRepository(ABC):
    @abstractmethod
    async def update_door_state(self, door_state: DoorState) -> None:
        """
        Update or create a door state.
        """

    @abstractmethod
    async def read_door_states(self) -> Dict[str, DoorState]:
        """
        Read the list of all current door states.
        """

    @abstractmethod
    async def update_door(self, door: Door) -> None:
        """
        Update or create a door.
        """

    @abstractmethod
    async def read_door(self, name: str) -> Optional[Door]:
        pass

    @abstractmethod
    async def sync_doors(self, doors: Sequence[Door]) -> None:
        """
        Update or create a list of doors and remove current doors not inside the list.
        """

    @abstractmethod
    async def update_door_health(self, door_health: DoorHealth) -> None:
        """
        Update or create door health.
        """

    @abstractmethod
    async def read_door_health(self, door_name: str) -> Optional[DoorHealth]:
        """
        Read the current door health
        """

    @abstractmethod
    async def update_lift_state(self, lift_state: LiftState):
        """
        Update or create a lift state
        """

    @abstractmethod
    async def read_lift_states(self) -> Dict[str, LiftState]:
        """
        Read the list of all current lift states.
        """

    @abstractmethod
    async def update_lift_health(self, lift_health: LiftHealth) -> None:
        """
        Update or create lift health.
        """

    @abstractmethod
    async def read_lift_health(self, lift_name: str) -> Optional[LiftHealth]:
        """
        Read the current lift health
        """

    @abstractmethod
    async def update_dispenser_state(self, dispenser_state: DispenserState):
        """
        Update or create a dispenser state
        """

    @abstractmethod
    async def read_dispenser_states(self) -> Dict[str, DispenserState]:
        """
        Read the list of all current dispenser states.
        """
