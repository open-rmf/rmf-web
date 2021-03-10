import json
import logging
from typing import Any, Callable, List, Optional, Sequence, Tuple

import rx
from building_map_msgs.msg import BuildingMap, Door, Level, Lift
from rmf_door_msgs.msg import DoorMode, DoorState
from rx import Observable
from rx import operators as op
from rx.core.typing import Disposable
from rx.scheduler.scheduler import Scheduler

from ..models import BasicHealthModel, DoorHealth, HealthStatus, LiftHealth
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
        self.watchers: List[Disposable] = []

        def on_building_map(building_map: Optional[BuildingMap]):
            for sub in self.watchers:
                sub.dispose()
            self.watchers.clear()
            self._watch_door_health(building_map)
            self._watch_lift_health(building_map)

        self.rmf.building_map.subscribe(on_building_map)

    def _report_health(self, target: Observable):
        def on_next(health: BasicHealthModel):
            message = json.dumps(
                {
                    "id": health.id_,
                    "health_status": str(health.health_status),
                    "health_message": health.health_message,
                }
            )
            if health.health_status == HealthStatus.UNHEALTHY:
                self.logger.warning(message)
            elif health.health_status == HealthStatus.DEAD:
                self.logger.error(message)
            else:
                self.logger.info(message)
            target.on_next(health)

        return on_next

    def _watch_heartbeat(
        self,
        key_mapper: Callable[[Any], str],
    ) -> Observable:
        """
        :returns: Observable[Tuple[str, bool]]
        """
        return rx.pipe(
            op.group_by(key_mapper),
            op.flat_map(
                lambda x: x.pipe(
                    heartbeat(self.LIVELINESS),
                    op.map(lambda y: (x.key, y)),
                )
            ),
        )

    def _watch_door_health(self, building_map):
        def door_mode_to_health(data: Tuple[str, DoorState]):
            state = data[1]
            # default to healthy if state is unknown
            if state is None:
                return DoorHealth(
                    id_=data[0],
                    health_status=HealthStatus.HEALTHY,
                )
            if state.current_mode.value == DoorMode.MODE_OFFLINE:
                return DoorHealth(
                    id_=state.door_name,
                    health_status=HealthStatus.UNHEALTHY,
                    health_message="offline",
                )
            if state.current_mode.value == DoorMode.MODE_UNKNOWN:
                return DoorHealth(
                    id_=state.door_name,
                    health_status=HealthStatus.UNHEALTHY,
                    health_message="unknown",
                )
            return DoorHealth(
                id_=state.door_name,
                health_status=HealthStatus.HEALTHY,
            )

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
                return DoorHealth(id_=id_, health_status=HealthStatus.HEALTHY)
            return DoorHealth(
                id_=id_,
                health_status=HealthStatus.DEAD,
                health_message="heartbeat failed",
            )

        keys = [x.name for x in doors]
        initial_values: Sequence[Tuple[str, Any]] = [(k, None) for k in keys]
        obs = rx.merge(
            rx.of(*initial_values),
            self.rmf.door_states.pipe(op.map(lambda x: (x.door_name, x))),
        )

        door_mode_health = obs.pipe(
            op.map(door_mode_to_health),
            op.timestamp(),
        )

        heartbeat_health = obs.pipe(
            self._watch_heartbeat(lambda x: x[0]),
            op.map(to_door_health),
            op.timestamp(),
        )

        sub = heartbeat_health.pipe(
            op.combine_latest(door_mode_health),
            most_critical(),
        ).subscribe(self._report_health(self.rmf.door_health), scheduler=self.scheduler)
        self.watchers.append(sub)

    def _watch_lift_health(self, building_map):
        lifts: Sequence[Lift] = building_map.lifts if building_map else []

        def to_lift_health(data: Tuple[str, bool]):
            id_ = data[0]
            has_heartbeat = data[1]
            if has_heartbeat:
                return LiftHealth(id_=id_, health_status=HealthStatus.HEALTHY)
            return LiftHealth(
                id_=id_,
                health_status=HealthStatus.DEAD,
                health_message="heartbeat failed",
            )

        keys = [x.name for x in lifts]
        initial_values: Sequence[Tuple[str, Any]] = [(k, None) for k in keys]
        sub = (
            rx.merge(
                rx.of(*initial_values),
                self.rmf.lift_states.pipe(op.map(lambda x: (x.lift_name, x))),
            )
            .pipe(
                self._watch_heartbeat(lambda x: x[0]),
                op.map(to_lift_health),
            )
            .subscribe(
                self._report_health(self.rmf.lift_health),
                scheduler=self.scheduler,
            )
        )
        self.watchers.append(sub)
