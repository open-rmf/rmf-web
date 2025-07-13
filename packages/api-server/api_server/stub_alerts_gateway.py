from api_server.fast_io.singleton_dep import singleton_dep


class StubAlertsGateway:
    def __init__(self):
        pass

    async def __aexit__(self, *exc):
        pass

    def respond_to_alert(self, alert_id: str, response: str):
        pass


@singleton_dep
def get_alerts_gateway():
    return StubAlertsGateway()
