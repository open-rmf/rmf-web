from typing import Any, Callable

from fastapi import Depends
from std_msgs.msg import Bool as RmfBool

from ..fast_io import FastIORouter
from ..gateway import RmfGateway
from ..repositories import RmfRepository
from ..rmf_io import RmfEvents


class FireAlarmRouter(FastIORouter):
    def __init__(
        self,
        rmf_gateway_dep: Callable[[], RmfGateway],
    ):
        super().__init__(tags=["FireAlarm"])

        @self.post("/request")
        async def post_fire_alarm_request(
            alarm: bool,
            ros_node: RmfGateway = Depends(rmf_gateway_dep),
        ):

            ros_node.fire_alarm_trigger.publish(RmfBool(data=alarm))
