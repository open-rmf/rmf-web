import logging
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import rx
from rmf_building_map_msgs.msg import BuildingMap, Door, Level, Lift
from rmf_dispenser_msgs.msg import DispenserState
from rmf_door_msgs.msg import DoorMode, DoorState
from rmf_fleet_msgs.msg import FleetState, RobotMode, RobotState
from rmf_ingestor_msgs.msg import IngestorState
from rmf_lift_msgs.msg import LiftState
from rx import Observable
from rx import operators as ops
from rx.core.typing import Disposable
from rx.scheduler.scheduler import Scheduler
from rx.subject import Subject

from ..models import tortoise_models as ttm
from .gateway import RmfGateway
from .operators import heartbeat, most_critical


class HealthWatchdog:
    LIVELINESS = 10

    def __init__(
        self,
        rmf_gateway: RmfGateway,
        *,
        scheduler: Optional[Scheduler] = None,
        logger: logging.Logger = None,
    ):
        self.rmf = rmf_gateway
        self.scheduler = scheduler
        self.logger = logger or logging.getLogger(self.__class__.__name__)
        self._watchers: List[Disposable] = []
        self._building_watchers: List[Disposable] = []
        self._started = False

    def start(self):
        if self._started:
            return

        def on_building_map(building_map: Optional[BuildingMap]):
            for sub in self._building_watchers:
                sub.dispose()
            self._building_watchers.clear()
            self._watch_door_health(building_map)
            self._watch_lift_health(building_map)

        self.rmf.rmf_building_map.subscribe(on_building_map)

        self._watch_dispenser_health()
        self._watch_ingestor_health()
        self._watch_robot_health()
        self._started = True

    def stop(self):
        for sub in self._watchers:
            sub.dispose()
        self._watchers.clear()
        for sub in self._building_watchers:
            sub.dispose()
        self._building_watchers.clear()

        self._started = False

    def _watch_heartbeat(
        self,
        key_mapper: Callable[[Any], str],
    ) -> Observable:
        """
        :returns: Observable[Tuple[str, bool]]
        """
        return rx.pipe(
            ops.group_by(key_mapper),
            ops.flat_map(
                lambda x: x.pipe(
                    heartbeat(self.LIVELINESS),
                    ops.map(lambda y: (x.key, y)),
                )
            ),
        )

    @staticmethod
    def _combine_most_critical(*obs: Sequence[rx.Observable]):
        """
        Combines an observable sequence of an observable sequence of BasicHealthModel to an
        observable sequence of BasicHealthModel with the most critical health status. If there
        are multiple BasicHealthModel with the same criticality, the most recent item is
        chosen.

        :param obs: Sequence[rx.Observable[BasicHealthModel]]
        """
        return rx.pipe(
            ops.timestamp(),
            ops.combine_latest(*[x.pipe(ops.timestamp()) for x in obs]),
            most_critical(),
        )

    @staticmethod
    def _door_mode_to_health(data: Tuple[str, DoorState]):
        state = data[1]
        if state is None:
            return ttm.DoorHealth(
                id_=data[0],
                health_status=ttm.HealthStatus.UNHEALTHY,
                health_message="no state available",
            )
        if state.current_mode.value in (
            DoorMode.MODE_CLOSED,
            DoorMode.MODE_MOVING,
            DoorMode.MODE_OPEN,
        ):
            return ttm.DoorHealth(
                id_=state.door_name,
                health_status=ttm.HealthStatus.HEALTHY,
            )
        if state.current_mode.value == DoorMode.MODE_OFFLINE:
            return ttm.DoorHealth(
                id_=state.door_name,
                health_status=ttm.HealthStatus.UNHEALTHY,
                health_message="door is OFFLINE",
            )
        return ttm.DoorHealth(
            id_=state.door_name,
            health_status=ttm.HealthStatus.UNHEALTHY,
            health_message="door is in an unknown mode",
        )

    def _watch_door_health(self, building_map):
        doors: List[Door] = []
        if building_map:
            for level in building_map.levels:
                level: Level
                for door in level.doors:
                    doors.append(door)

        def to_door_health(data: Tuple[str, bool]):
            id_ = data[0]
            has_heartbeat = data[1]
            if has_heartbeat:
                return ttm.DoorHealth(id_=id_, health_status=ttm.HealthStatus.HEALTHY)
            return ttm.DoorHealth(
                id_=id_,
                health_status=ttm.HealthStatus.DEAD,
                health_message="heartbeat failed",
            )

        keys = [x.name for x in doors]
        initial_values: Sequence[Tuple[str, Any]] = [(k, None) for k in keys]
        obs = rx.merge(
            rx.of(*initial_values),
            self.rmf.door_states.pipe(ops.map(lambda x: (x.door_name, x))),
        )

        door_mode_health = obs.pipe(ops.map(self._door_mode_to_health))

        heartbeat_health = obs.pipe(
            self._watch_heartbeat(lambda x: x[0]),
            ops.map(to_door_health),
        )

        sub = heartbeat_health.pipe(
            self._combine_most_critical(door_mode_health),
        ).subscribe(self.rmf.door_health.on_next, scheduler=self.scheduler)
        self._building_watchers.append(sub)

    @staticmethod
    def _lift_mode_to_health(data: Tuple[str, LiftState]):
        state = data[1]
        if state is None:
            return ttm.LiftHealth(
                id_=data[0],
                health_status=ttm.HealthStatus.UNHEALTHY,
                health_message="no state available",
            )
        if state.current_mode in (
            LiftState.MODE_HUMAN,
            LiftState.MODE_AGV,
        ):
            return ttm.LiftHealth(
                id_=state.lift_name,
                health_status=ttm.HealthStatus.HEALTHY,
            )
        if state.current_mode == LiftState.MODE_FIRE:
            return ttm.LiftHealth(
                id_=state.lift_name,
                health_status=ttm.HealthStatus.UNHEALTHY,
                health_message="lift is in FIRE mode",
            )
        if state.current_mode == LiftState.MODE_EMERGENCY:
            return ttm.LiftHealth(
                id_=state.lift_name,
                health_status=ttm.HealthStatus.UNHEALTHY,
                health_message="lift is in EMERGENCY mode",
            )
        return ttm.LiftHealth(
            id_=state.lift_name,
            health_status=ttm.HealthStatus.UNHEALTHY,
            health_message="lift is in an unknown mode",
        )

    def _watch_lift_health(self, building_map):
        lifts: Sequence[Lift] = building_map.lifts if building_map else []

        def to_lift_health(data: Tuple[str, bool]):
            id_ = data[0]
            has_heartbeat = data[1]
            if has_heartbeat:
                return ttm.LiftHealth(id_=id_, health_status=ttm.HealthStatus.HEALTHY)
            return ttm.LiftHealth(
                id_=id_,
                health_status=ttm.HealthStatus.DEAD,
                health_message="heartbeat failed",
            )

        keys = [x.name for x in lifts]
        initial_values: Sequence[Tuple[str, Any]] = [(k, None) for k in keys]
        obs = rx.merge(
            rx.of(*initial_values),
            self.rmf.lift_states.pipe(ops.map(lambda x: (x.lift_name, x))),
        )

        lift_mode_health = obs.pipe(ops.map(self._lift_mode_to_health))
        heartbeat_health = obs.pipe(
            self._watch_heartbeat(lambda x: x[0]),
            ops.map(to_lift_health),
        )
        sub = heartbeat_health.pipe(
            self._combine_most_critical(lift_mode_health)
        ).subscribe(self.rmf.lift_health.on_next, scheduler=self.scheduler)
        self._building_watchers.append(sub)

    @staticmethod
    def _dispenser_mode_to_health(id_: str, state: DispenserState):
        if state is None:
            return ttm.DispenserHealth(
                id_=id_,
                health_status=ttm.HealthStatus.UNHEALTHY,
                health_message="no state available",
            )
        if state.mode in (
            DispenserState.IDLE,
            DispenserState.BUSY,
        ):
            return ttm.DispenserHealth(
                id_=id_,
                health_status=ttm.HealthStatus.HEALTHY,
            )
        if state.mode == DispenserState.OFFLINE:
            return ttm.DispenserHealth(
                id_=id_,
                health_status=ttm.HealthStatus.UNHEALTHY,
                health_message="dispenser is OFFLINE",
            )
        return ttm.DispenserHealth(
            id_=id_,
            health_status=ttm.HealthStatus.UNHEALTHY,
            health_message="dispenser is in an unknown mode",
        )

    def _watch_dispenser_health(self):
        def to_dispenser_health(id_: str, has_heartbeat: bool):
            if has_heartbeat:
                return ttm.DispenserHealth(
                    id_=id_, health_status=ttm.HealthStatus.HEALTHY
                )
            return ttm.DispenserHealth(
                id_=id_,
                health_status=ttm.HealthStatus.DEAD,
                health_message="heartbeat failed",
            )

        def watch(id_: str, obs: Observable):
            dispenser_mode_health = obs.pipe(
                ops.map(lambda x: self._dispenser_mode_to_health(id_, x))
            )
            obs.pipe(
                heartbeat(self.LIVELINESS),
                ops.map(lambda x: to_dispenser_health(id_, x)),
                self._combine_most_critical(dispenser_mode_health),
            ).subscribe(self.rmf.dispenser_health.on_next, scheduler=self.scheduler)

        subjects = {
            x.guid: Subject() for x in self.rmf.current_dispenser_states.values()
        }
        for guid, subject in subjects.items():
            watch(guid, subject)

        def on_state(state: DispenserState):
            if state.guid not in subjects:
                subjects[state.guid] = Subject()
                watch(state.guid, subjects[state.guid])
            subjects[state.guid].on_next(state)

        self.rmf.dispenser_states.subscribe(on_state)

    @staticmethod
    def _ingestor_mode_to_health(id_: str, state: IngestorState):
        if state is None:
            return ttm.IngestorHealth(
                id_=id_,
                health_status=ttm.HealthStatus.UNHEALTHY,
                health_message="no state available",
            )
        if state.mode in (
            IngestorState.IDLE,
            IngestorState.BUSY,
        ):
            return ttm.IngestorHealth(
                id_=id_,
                health_status=ttm.HealthStatus.HEALTHY,
            )
        if state.mode == IngestorState.OFFLINE:
            return ttm.IngestorHealth(
                id_=id_,
                health_status=ttm.HealthStatus.UNHEALTHY,
                health_message="ingestor is OFFLINE",
            )
        return ttm.IngestorHealth(
            id_=id_,
            health_status=ttm.HealthStatus.UNHEALTHY,
            health_message="ingestor is in an unknown mode",
        )

    def _watch_ingestor_health(self):
        def to_ingestor_health(id_: str, has_heartbeat: bool):
            if has_heartbeat:
                return ttm.IngestorHealth(
                    id_=id_, health_status=ttm.HealthStatus.HEALTHY
                )
            return ttm.IngestorHealth(
                id_=id_,
                health_status=ttm.HealthStatus.DEAD,
                health_message="heartbeat failed",
            )

        def watch(id_: str, obs: Observable):
            ingestor_mode_health = obs.pipe(
                ops.map(lambda x: self._ingestor_mode_to_health(id_, x))
            )
            obs.pipe(
                heartbeat(self.LIVELINESS),
                ops.map(lambda x: to_ingestor_health(id_, x)),
                self._combine_most_critical(ingestor_mode_health),
            ).subscribe(self.rmf.ingestor_health.on_next, scheduler=self.scheduler)

        subjects = {
            x.guid: Subject() for x in self.rmf.current_ingestor_states.values()
        }
        for guid, subject in subjects.items():
            watch(guid, subject)

        def on_state(state: IngestorState):
            if state.guid not in subjects:
                subjects[state.guid] = Subject()
                watch(state.guid, subjects[state.guid])
            subjects[state.guid].on_next(state)

        self.rmf.ingestor_states.subscribe(on_state)

    @staticmethod
    def _robot_mode_to_health(id_: str, state: RobotState):
        if state is None:
            return ttm.RobotHealth(
                id_=id_,
                health_status=ttm.HealthStatus.UNHEALTHY,
                health_message="no state available",
            )
        if state.mode.mode in (
            RobotMode.MODE_IDLE,
            RobotMode.MODE_CHARGING,
            RobotMode.MODE_MOVING,
            RobotMode.MODE_PAUSED,
            RobotMode.MODE_WAITING,
            RobotMode.MODE_GOING_HOME,
            RobotMode.MODE_DOCKING,
        ):
            return ttm.RobotHealth(
                id_=id_,
                health_status=ttm.HealthStatus.HEALTHY,
            )
        if state.mode.mode == RobotMode.MODE_EMERGENCY:
            return ttm.RobotHealth(
                id_=id_,
                health_status=ttm.HealthStatus.UNHEALTHY,
                health_message="robot is in EMERGENCY mode",
            )
        if state.mode.mode == RobotMode.MODE_ADAPTER_ERROR:
            return ttm.RobotHealth(
                id_=id_,
                health_status=ttm.HealthStatus.UNHEALTHY,
                health_message="error in fleet adapter",
            )
        return ttm.RobotHealth(
            id_=id_,
            health_status=ttm.HealthStatus.UNHEALTHY,
            health_message="robot is in an unknown mode",
        )

    def _watch_robot_health(self):
        def to_robot_health(id_: str, has_heartbeat: bool):
            if has_heartbeat:
                return ttm.RobotHealth(
                    id_=id_,
                    health_status=ttm.HealthStatus.HEALTHY,
                )
            return ttm.RobotHealth(
                id_=id_,
                health_status=ttm.HealthStatus.DEAD,
                health_message="heartbeat failed",
            )

        def watch(id_: str, obs: Observable):
            """
            :param obs: Observable[RobotState]
            """
            robot_mode_health = obs.pipe(
                ops.map(lambda x: self._robot_mode_to_health(id_, x))
            )
            obs.pipe(
                heartbeat(self.LIVELINESS),
                ops.map(lambda x: to_robot_health(id_, x)),
                self._combine_most_critical(robot_mode_health),
            ).subscribe(self.rmf.robot_health.on_next, scheduler=self.scheduler)

        subjects: Dict[str, Subject] = {}
        for fleet_state in self.rmf.current_fleet_states.values():
            fleet_state: FleetState
            for robot_state in fleet_state.robots:
                robot_state: RobotState
                robot_id = ttm.get_robot_id(fleet_state.name, robot_state.name)
                subjects[robot_id] = Subject()

        for id_, subject in subjects.items():
            watch(id_, subject)

        def on_state(fleet_state: FleetState):
            for robot_state in fleet_state.robots:
                robot_state: RobotState
                robot_id = ttm.get_robot_id(fleet_state.name, robot_state.name)

                if robot_id not in subjects:
                    subjects[robot_id] = Subject()
                    watch(robot_id, subjects[robot_id])
                subjects[robot_id].on_next(robot_state)

        self.rmf.fleet_states.subscribe(on_state)
