from ..fast_io import FastIORouter
from ..models import ChargerRequest
from ..rmf_io import RmfEvents


class ChargersRouter(FastIORouter):
    def __init__(
        self,
        rmf_events: RmfEvents,
    ):
        super().__init__(tags=["Chargers"])

        @self.watch(
            "/{charger_name}/request",
            rmf_events.charger_requests,
            response_model=ChargerRequest,
        )
        def get_charger_request(charger_request: ChargerRequest):
            return {"charger_name": charger_request.charger_name}, charger_request
