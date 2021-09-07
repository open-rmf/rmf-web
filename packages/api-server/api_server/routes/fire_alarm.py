from typing import Any, Callable

from fastapi import Depends
from std_msgs.msg import Bool

from ..fast_io import FastIORouter
from ..gateway import RmfGateway
from ..repositories import RmfRepository
from ..rmf_io import RmfEvents


class FireAlarmRouter(FastIORouter):
    def __init__(
        self,
        rmf_events: RmfEvents,
        rmf_gateway_dep: Callable[[], RmfGateway],
    ):
        super().__init__(tags=["FireAlarm"])

        @self.post("/request")
        async def post_fire_alarm_request(
            alarm: Bool,
            ros_node: RmfGateway = Depends(rmf_gateway_dep),
        ):
            msg = Bool(alarm)
            ros_node.fire_alarm_trigger.publish(msg)
