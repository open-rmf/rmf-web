import logging
from typing import Any, Callable, Dict, Optional, TypeVar, cast

from reactivex import Observable, compose
from reactivex import operators as ops
from reactivex.scheduler.scheduler import Scheduler
from reactivex.subject.behaviorsubject import BehaviorSubject
from rmf_dispenser_msgs.msg import DispenserState as RmfDispenserState
from rmf_door_msgs.msg import DoorMode as RmfDoorMode
from rmf_ingestor_msgs.msg import IngestorState as RmfIngestorState
from rmf_lift_msgs.msg import LiftState as RmfLiftState
from tortoise.exceptions import MultipleObjectsReturned

from api_server.models import (
    BasicHealth,
    BuildingMap,
    Dispenser,
    DispenserHealth,
    DispenserState,
    DoorHealth,
    DoorState,
    HealthStatus,
    Ingestor,
    IngestorHealth,
    IngestorState,
    LiftHealth,
    LiftState,
)
from api_server.models import tortoise_models as ttm

from .events import RmfEvents
from .operators import filter_not_none, heartbeat, most_critical

T = TypeVar("T", bound=BasicHealth)


class HealthWatchdog:
    LIVELINESS = 10

    def __init__(
        self,
        rmf_events: RmfEvents,
        *,
        scheduler: Optional[Scheduler] = None,
        logger: Optional[logging.Logger] = None,
    ):
        self.rmf = rmf_events
        self.scheduler = scheduler
        self.logger = logger or logging.getLogger(self.__class__.__name__)

    async def start(self):
        await self._watch_door_health()
        await self._watch_lift_health()
        await self._watch_dispenser_health()
        await self._watch_ingestor_health()

    @staticmethod
    def _combine_most_critical(
        *obs: Observable[T | None],
    ) -> Callable[[Observable[T]], Observable[T]]:
        """
        Combines an observable sequence of an observable sequence of BasicHealthModel to an
        observable sequence of BasicHealthModel with the most critical health status. If there
        are multiple BasicHealthModel with the same criticality, the most recent item is
        chosen.
        """
        return compose(
            ops.timestamp(),
            cast(Any, ops.combine_latest(*[x.pipe(ops.timestamp()) for x in obs])),
            most_critical(),
            filter_not_none,
        )

    @staticmethod
    def door_mode_to_health(state: Optional[DoorState]) -> Optional[DoorHealth]:
        if state is None:
            return None

        if state.current_mode.value in (
            RmfDoorMode.MODE_CLOSED,
            RmfDoorMode.MODE_MOVING,
            RmfDoorMode.MODE_OPEN,
        ):
            return DoorHealth(
                id_=state.door_name,
                health_status=HealthStatus.HEALTHY,
                health_message="",
            )
        if state.current_mode.value == RmfDoorMode.MODE_OFFLINE:
            return DoorHealth(
                id_=state.door_name,
                health_status=HealthStatus.UNHEALTHY,
                health_message="door is OFFLINE",
            )
        return DoorHealth(
            id_=state.door_name,
            health_status=HealthStatus.UNHEALTHY,
            health_message="door is in an unknown mode",
        )

    async def _watch_door_health(self):
        def to_door_health(id_: str, has_heartbeat: bool):
            if has_heartbeat:
                return DoorHealth(
                    id_=id_, health_status=HealthStatus.HEALTHY, health_message=""
                )
            return DoorHealth(
                id_=id_,
                health_status=HealthStatus.DEAD,
                health_message="heartbeat failed",
            )

        def watch(id_: str, obs: Observable):
            door_mode_health = obs.pipe(
                ops.map(self.door_mode_to_health),
                ops.distinct_until_changed(),
            )
            obs.pipe(
                heartbeat(self.LIVELINESS),
                ops.map(lambda has_heartbeat: to_door_health(id_, has_heartbeat)),
                self._combine_most_critical(door_mode_health),
            ).subscribe(self.rmf.door_health.on_next, scheduler=self.scheduler)

        try:
            ttm_map = await ttm.BuildingMap.get_or_none()
        except MultipleObjectsReturned:
            ttm_maps = await ttm.BuildingMap.all()
            map_names = [BuildingMap.from_tortoise(m).name for m in ttm_maps]
            self.logger.error(
                "There appears to be multiple building maps "
                f"available: {map_names}. Please ensure that "
                "there is only a single building_map_server "
                "running in this deployment, start a fresh "
                "database or remove the rogue map, before "
                "starting the API server again."
            )
            raise

        if ttm_map is None:
            doors = []
        else:
            building_map = BuildingMap.from_tortoise(ttm_map)
            doors = [door for level in building_map.levels for door in level.doors]
        states_list = [DoorState.from_tortoise(x) for x in await ttm.DoorState.all()]
        door_states = {state.door_name: state for state in states_list}
        initial_states = {door.name: door_states.get(door.name, None) for door in doors}

        subjects: Dict[str, BehaviorSubject] = {
            id_: BehaviorSubject(state) for id_, state in initial_states.items()
        }
        for id_, subject in subjects.items():
            watch(id_, subject)

        def on_state(state: DoorState):
            if state.door_name not in subjects:
                subjects[state.door_name] = BehaviorSubject(state)
                watch(state.door_name, subjects[state.door_name])
            else:
                subjects[state.door_name].on_next(state)

        self.rmf.door_states.subscribe(on_state)

    @staticmethod
    def lift_mode_to_health(state: Optional[LiftState]):
        if state is None:
            return None

        if state.current_mode in (
            RmfLiftState.MODE_HUMAN,
            RmfLiftState.MODE_AGV,
        ):
            return LiftHealth(
                id_=state.lift_name,
                health_status=HealthStatus.HEALTHY,
                health_message="",
            )
        if state.current_mode == RmfLiftState.MODE_FIRE:
            return LiftHealth(
                id_=state.lift_name,
                health_status=HealthStatus.UNHEALTHY,
                health_message="lift is in FIRE mode",
            )
        if state.current_mode == RmfLiftState.MODE_EMERGENCY:
            return LiftHealth(
                id_=state.lift_name,
                health_status=HealthStatus.UNHEALTHY,
                health_message="lift is in EMERGENCY mode",
            )
        return LiftHealth(
            id_=state.lift_name,
            health_status=HealthStatus.UNHEALTHY,
            health_message="lift is in an unknown mode",
        )

    async def _watch_lift_health(self):
        def to_lift_health(id_: str, has_heartbeat: bool):
            if has_heartbeat:
                return LiftHealth(
                    id_=id_, health_status=HealthStatus.HEALTHY, health_message=""
                )
            return LiftHealth(
                id_=id_,
                health_status=HealthStatus.DEAD,
                health_message="heartbeat failed",
            )

        def watch(id_: str, obs: Observable):
            lift_mode_health = obs.pipe(
                ops.map(self.lift_mode_to_health),
                ops.distinct_until_changed(),
            )
            obs.pipe(
                heartbeat(self.LIVELINESS),
                ops.map(lambda has_heartbeat: to_lift_health(id_, has_heartbeat)),
                self._combine_most_critical(lift_mode_health),
            ).subscribe(self.rmf.lift_health.on_next, scheduler=self.scheduler)

        ttm_map = await ttm.BuildingMap.get_or_none()
        if ttm_map is None:
            lifts = []
        else:
            building_map = BuildingMap.from_tortoise(ttm_map)
            lifts = building_map.lifts
        states_list = [LiftState.from_tortoise(x) for x in await ttm.LiftState.all()]
        lift_states = {state.lift_name: state for state in states_list}
        initial_states = {lift.name: lift_states.get(lift.name, None) for lift in lifts}

        subjects: Dict[str, BehaviorSubject] = {
            id_: BehaviorSubject(state) for id_, state in initial_states.items()
        }
        for id_, subject in subjects.items():
            watch(id_, subject)

        def on_state(state: LiftState):
            if state.lift_name not in subjects:
                subjects[state.lift_name] = BehaviorSubject(state)
                watch(state.lift_name, subjects[state.lift_name])
            else:
                subjects[state.lift_name].on_next(state)

        self.rmf.lift_states.subscribe(on_state)

    @staticmethod
    def dispenser_mode_to_health(state: Optional[DispenserState]):
        if state is None:
            return None
        if state.mode in (
            RmfDispenserState.IDLE,
            RmfDispenserState.BUSY,
        ):
            return DispenserHealth(
                id_=state.guid,
                health_status=HealthStatus.HEALTHY,
                health_message="",
            )
        if state.mode == RmfDispenserState.OFFLINE:
            return DispenserHealth(
                id_=state.guid,
                health_status=HealthStatus.UNHEALTHY,
                health_message="dispenser is OFFLINE",
            )
        return DispenserHealth(
            id_=state.guid,
            health_status=HealthStatus.UNHEALTHY,
            health_message="dispenser is in an unknown mode",
        )

    async def _watch_dispenser_health(self):
        def to_dispenser_health(id_: str, has_heartbeat: bool):
            if has_heartbeat:
                return DispenserHealth(
                    id_=id_, health_status=HealthStatus.HEALTHY, health_message=""
                )
            return DispenserHealth(
                id_=id_,
                health_status=HealthStatus.DEAD,
                health_message="heartbeat failed",
            )

        def watch(id_: str, obs: Observable):
            dispenser_mode_health = obs.pipe(
                ops.map(self.dispenser_mode_to_health),
                ops.distinct_until_changed(),
            )
            obs.pipe(
                heartbeat(self.LIVELINESS),
                ops.map(
                    lambda has_heartbeat: to_dispenser_health(id_, has_heartbeat),
                ),
                self._combine_most_critical(dispenser_mode_health),
            ).subscribe(self.rmf.dispenser_health.on_next, scheduler=self.scheduler)

        states_list = [
            DispenserState.from_tortoise(x) for x in await ttm.DispenserState.all()
        ]
        dispensers = [Dispenser(guid=x.guid) for x in states_list]
        dispenser_states = {state.guid: state for state in states_list}
        initial_states = {
            dispenser.guid: dispenser_states.get(dispenser.guid, None)
            for dispenser in dispensers
        }

        subjects: Dict[str, BehaviorSubject] = {
            id_: BehaviorSubject(state) for id_, state in initial_states.items()
        }
        for id_, subject in subjects.items():
            watch(id_, subject)

        def on_state(state: DispenserState):
            if state.guid not in subjects:
                subjects[state.guid] = BehaviorSubject(state)
                watch(state.guid, subjects[state.guid])
            else:
                subjects[state.guid].on_next(state)

        self.rmf.dispenser_states.subscribe(on_state)

    @staticmethod
    def ingestor_mode_to_health(state: IngestorState):
        if state.mode in (
            RmfIngestorState.IDLE,
            RmfIngestorState.BUSY,
        ):
            return IngestorHealth(
                id_=state.guid, health_status=HealthStatus.HEALTHY, health_message=""
            )
        if state.mode == RmfIngestorState.OFFLINE:
            return IngestorHealth(
                id_=state.guid,
                health_status=HealthStatus.UNHEALTHY,
                health_message="ingestor is OFFLINE",
            )
        return IngestorHealth(
            id_=state.guid,
            health_status=HealthStatus.UNHEALTHY,
            health_message="ingestor is in an unknown mode",
        )

    async def _watch_ingestor_health(self):
        def to_ingestor_health(id_: str, has_heartbeat: bool):
            if has_heartbeat:
                return IngestorHealth(
                    id_=id_, health_status=HealthStatus.HEALTHY, health_message=""
                )
            return IngestorHealth(
                id_=id_,
                health_status=HealthStatus.DEAD,
                health_message="heartbeat failed",
            )

        def watch(id_: str, obs: Observable):
            ingestor_mode_health = obs.pipe(
                ops.map(self.ingestor_mode_to_health),
                ops.distinct_until_changed(),
            )
            obs.pipe(
                heartbeat(self.LIVELINESS),
                ops.map(
                    lambda has_heartbeat: to_ingestor_health(id_, has_heartbeat),
                ),
                self._combine_most_critical(ingestor_mode_health),
            ).subscribe(self.rmf.ingestor_health.on_next, scheduler=self.scheduler)

        states_list = [
            IngestorState.from_tortoise(x) for x in await ttm.IngestorState.all()
        ]
        ingestors = [Ingestor(guid=x.guid) for x in states_list]
        ingestor_states = {state.guid: state for state in states_list}
        initial_states = {
            ingestor.guid: ingestor_states.get(ingestor.guid, None)
            for ingestor in ingestors
        }

        subjects: Dict[str, BehaviorSubject] = {
            id_: BehaviorSubject(state) for id_, state in initial_states.items()
        }
        for id_, subject in subjects.items():
            watch(id_, subject)

        def on_state(state: IngestorState):
            if state.guid not in subjects:
                subjects[state.guid] = BehaviorSubject(state)
                watch(state.guid, subjects[state.guid])
            else:
                subjects[state.guid].on_next(state)

        self.rmf.ingestor_states.subscribe(on_state)
