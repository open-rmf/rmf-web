from typing import Callable

from fastapi import Depends
from rmf_charger_msgs.msg import ChargerRequest as RmfChargerRequest

from ..fast_io import FastIORouter
from ..gateway import RmfGateway
from ..models import ChargerRequest, ChargerState, FleetState, TaskSummary
from ..rmf_io import RmfEvents


class ChargersRouter(FastIORouter):
    def __init__(
        self,
        rmf_events: RmfEvents,
        rmf_gateway_dep: Callable[[], RmfGateway],
    ):
        super().__init__(tags=["Chargers"])

        @self.watch(
            "/{charger_name}/request",
            rmf_events.charger_requests,
            response_model=ChargerRequest,
        )
        def get_charger_request(charger_request: ChargerRequest):
            return {"charger_name": charger_request.charger_name}, charger_request
