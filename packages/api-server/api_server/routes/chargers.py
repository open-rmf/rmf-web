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

        @self.get("")
        async def get_chargers():
            print("charges is hitttt !!!!!!!!!!")
            return "this is charger"

        @self.watch(
            "/{charger_name}/state",
            rmf_events.charger_states,
            response_model=ChargerState,
        )
        def get_charger_state(charger_state: ChargerState):
            print("I am hit")
            return {"name": charger_state.charger_name}, charger_state

        # @self.post("/{charger_name}/ruest")
        # async def post_charger_request(
        #   charger_name: str,
        #   charger_request: ChargerRequest,
        #   ros_node: RmfGateway = Depends(rmf_gateway_dep),
        # ):
        #   print("testing")
        #   msg = RmfChargerRequest(
        #     charger_name=charger_name
        # )
