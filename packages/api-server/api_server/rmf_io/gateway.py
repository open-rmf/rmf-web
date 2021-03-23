from typing import Any, Callable, Dict

from rmf_dispenser_msgs.msg import DispenserState
from rmf_door_msgs.msg import DoorState
from rmf_fleet_msgs.msg import FleetState
from rmf_ingestor_msgs.msg import IngestorState
from rmf_lift_msgs.msg import LiftState
from rmf_task_msgs.msg import TaskSummary
from rx import Observable
from rx.subject import BehaviorSubject, Subject

from .. import models


class RmfGateway:
    """
    NOTE: Observables that contains tortoise models might not be synchronized to the database,
    this means that calling certain functions are undefined behavior. Below are a list
    of methods that should not be called:

    * `save`. `save` requires the model to be synchronized to the db to correctly determine
      if it should update or create a new record. Instead, use `Model.update_or_create`.
    """

    def __init__(
        self,
    ):
        # NOTE: the rx type hints don't actually work https://github.com/ReactiveX/RxPY/issues/514
        self.door_states = Subject()  # Subject[DoorState]
        self.current_door_states: Dict[str, DoorState] = {}
        self._save_event(
            self.door_states,
            self.current_door_states,
            lambda x: x.door_name,
        )

        self.door_health = Subject()  # Subject[DoorHealth]
        self.current_door_health: Dict[str, models.DoorHealth] = {}
        self._save_event(
            self.door_health,
            self.current_door_health,
            lambda x: x.id_,
        )

        self.lift_states = Subject()  # Subject[LiftState]
        self.current_lift_states: Dict[str, LiftState] = {}
        self._save_event(
            self.lift_states,
            self.current_lift_states,
            lambda x: x.lift_name,
        )

        self.lift_health = Subject()  # Subject[LiftHealth]
        self.current_lift_health: Dict[str, models.LiftHealth] = {}
        self._save_event(
            self.lift_health,
            self.current_lift_health,
            lambda x: x.id_,
        )

        self.dispenser_states = Subject()  # Subject[DispenserState]
        self.current_dispenser_states: Dict[str, DispenserState] = {}
        self._save_event(
            self.dispenser_states,
            self.current_dispenser_states,
            lambda x: x.guid,
        )

        self.dispenser_health = Subject()  # Subject[DispenserHealth]
        self.current_dispenser_health: Dict[str, models.DispenserHealth] = {}
        self._save_event(
            self.dispenser_health,
            self.current_dispenser_health,
            lambda x: x.id_,
        )

        self.ingestor_states = Subject()  # Subject[IngestorState]
        self.current_ingestor_states: Dict[str, IngestorState] = {}
        self._save_event(
            self.ingestor_states,
            self.current_ingestor_states,
            lambda x: x.guid,
        )

        self.ingestor_health = Subject()  # Subject[IngestorHealth]
        self.current_ingestor_health: Dict[str, models.IngestorHealth] = {}
        self._save_event(
            self.ingestor_health,
            self.current_ingestor_health,
            lambda x: x.id_,
        )

        self.fleet_states = Subject()  # Subject[FleetState]
        self.current_fleet_states: Dict[str, FleetState] = {}
        self._save_event(
            self.fleet_states,
            self.current_fleet_states,
            lambda x: x.name,
        )

        self.robot_health = Subject()  # Subject[RobotHealth]
        self.current_robot_health: Dict[str, models.RobotHealth] = {}
        self._save_event(
            self.robot_health,
            self.current_robot_health,
            lambda x: x.id_,
        )

        self.task_summaries = Subject()  # Subject[TaskSummary]
        self.current_task_summaries: Dict[str, TaskSummary] = {}
        self._init_task_summaries()

        self.building_map = BehaviorSubject(  # BehaviorSubject[Optional[BuildingMap]]
            None
        )

    @staticmethod
    def _save_event(source: Observable, dic, key_mapper: Callable[[Any], str]):
        def on_next(data):
            dic[key_mapper(data)] = data

        source.subscribe(on_next)

    def _init_task_summaries(self):
        def on_next(task: TaskSummary):
            keep_states = models.TaskSummary.ACTIVE_STATES
            if task.state in keep_states:
                self.current_task_summaries[task.task_id] = task
            else:
                self.current_task_summaries.pop(task.task_id)

        self.task_summaries.subscribe(on_next)
