import asyncio
import json
import logging

from rmf_dispenser_msgs.msg import DispenserState
from rmf_door_msgs.msg import DoorState
from rmf_fleet_msgs.msg import FleetState
from rmf_lift_msgs.msg import LiftState
from rosidl_runtime_py.convert import message_to_ordereddict

from .. import models
from .gateway import RmfGateway


class RmfBookKeeper:
    def __init__(
        self,
        rmf_gateway: RmfGateway,
        *,
        loop: asyncio.AbstractEventLoop = None,
        logger: logging.Logger = None,
    ):
        self.rmf = rmf_gateway
        self.loop: asyncio.AbstractEventLoop = loop or asyncio.get_event_loop()
        self.logger = logger or logging.getLogger(self.__class__.__name__)

    def start(
        self,
    ):
        self._record_door_state()
        self._record_door_health()
        self._record_lift_state()
        self._record_lift_health()
        self._record_dispenser_state()
        self._record_dispenser_health()
        self._record_fleet_state()
        self._record_robot_health()

    def _report_health(self, health: models.BasicHealthModel):
        message = json.dumps(
            {
                "id": health.id_,
                "health_status": str(health.health_status),
                "health_message": health.health_message,
            }
        )
        if health.health_status == models.HealthStatus.UNHEALTHY:
            self.logger.warning(message)
        elif health.health_status == models.HealthStatus.DEAD:
            self.logger.error(message)
        else:
            self.logger.info(message)

    def _record_door_state(self):
        async def update(door_state: DoorState):
            await models.DoorState.update_or_create_from_rmf(door_state)
            self.logger.info(json.dumps(message_to_ordereddict(door_state)))

        self.rmf.door_states.subscribe(lambda x: self.loop.create_task(update(x)))

    def _record_door_health(self):
        async def update(health: models.DoorHealth):
            await models.DoorHealth.update_or_create(
                {
                    "health_status": health.health_status,
                    "health_message": health.health_message,
                },
                id_=health.id_,
            )
            self._report_health(health)

        self.rmf.door_health.subscribe(lambda x: self.loop.create_task(update(x)))

    def _record_lift_state(self):
        async def update(lift_state: LiftState):
            await models.LiftState.update_or_create_from_rmf(lift_state)
            self.logger.info(json.dumps(message_to_ordereddict(lift_state)))

        self.rmf.lift_states.subscribe(lambda x: self.loop.create_task(update(x)))

    def _record_lift_health(self):
        async def update(health: models.LiftHealth):
            await models.LiftHealth.update_or_create(
                {
                    "health_status": health.health_status,
                    "health_message": health.health_message,
                },
                id_=health.id_,
            )
            self._report_health(health)

        self.rmf.lift_health.subscribe(lambda x: self.loop.create_task(update(x)))

    def _record_dispenser_state(self):
        async def update(dispenser_state: DispenserState):
            await models.DispenserState.update_or_create_from_rmf(dispenser_state)
            self.logger.info(json.dumps(message_to_ordereddict(dispenser_state)))

        self.rmf.dispenser_states.subscribe(lambda x: self.loop.create_task(update(x)))

    def _record_dispenser_health(self):
        async def update(health: models.DispenserHealth):
            await models.DispenserHealth.update_or_create(
                {
                    "health_status": health.health_status,
                    "health_message": health.health_message,
                },
                id_=health.id_,
            )
            self._report_health(health)

        self.rmf.dispenser_health.subscribe(lambda x: self.loop.create_task(update(x)))

    def _record_fleet_state(self):
        async def update(fleet_state: FleetState):
            await models.FleetState.update_or_create_from_rmf(fleet_state)
            self.logger.info(json.dumps(message_to_ordereddict(fleet_state)))

        self.rmf.fleet_states.subscribe(lambda x: self.loop.create_task(update(x)))

    def _record_robot_health(self):
        async def update(health: models.RobotHealth):
            await models.RobotHealth.update_or_create(
                {
                    "health_status": health.health_status,
                    "health_message": health.health_message,
                },
                id_=health.id_,
            )
            self._report_health(health)

        self.rmf.robot_health.subscribe(lambda x: self.loop.create_task(update(x)))
