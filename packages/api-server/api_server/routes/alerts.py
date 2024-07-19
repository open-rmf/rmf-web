from typing import Annotated

from fastapi import Depends, HTTPException
from reactivex import operators as rxops

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import Alert
from api_server.repositories import AlertRepository
from api_server.rmf_io import AlertEvents
from api_server.rmf_io.events import get_alert_events

router = FastIORouter(tags=["Alerts"])


@router.sub("", response_model=Alert)
async def sub_alerts(
    _req: SubscriptionRequest,
):
    alert_events = get_alert_events()
    return alert_events.alerts.pipe(rxops.filter(lambda x: x is not None))


@router.get("", response_model=list[Alert])
async def get_alerts(repo: Annotated[AlertRepository, Depends(AlertRepository)]):
    return await repo.get_all_alerts()


@router.get("/{alert_id}", response_model=Alert)
async def get_alert(
    alert_id: str, repo: Annotated[AlertRepository, Depends(AlertRepository)]
):
    alert = await repo.get_alert(alert_id)
    if alert is None:
        raise HTTPException(404, f"Alert with ID {alert_id} not found")
    return alert


@router.post("", status_code=201, response_model=Alert)
async def create_alert(
    alert_id: str,
    category: str,
    repo: Annotated[AlertRepository, Depends(AlertRepository)],
):
    alert = await repo.create_alert(alert_id, category)
    if alert is None:
        raise HTTPException(404, f"Could not create alert with ID {alert_id}")
    return alert


@router.post("/{alert_id}", status_code=201, response_model=Alert)
async def acknowledge_alert(
    alert_id: str,
    repo: Annotated[AlertRepository, Depends(AlertRepository)],
    alert_events: Annotated[AlertEvents, Depends(get_alert_events)],
):
    alert = await repo.acknowledge_alert(alert_id)
    if alert is None:
        raise HTTPException(404, f"Could acknowledge alert with ID {alert_id}")
    alert_events.alerts.on_next(alert)
    return alert
