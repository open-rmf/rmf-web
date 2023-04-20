from typing import List

from fastapi import Depends, HTTPException
from rx import operators as rxops

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import tortoise_models as ttm
from api_server.repositories import AlertRepository, alert_repo_dep
from api_server.rmf_io import alert_events

router = FastIORouter(tags=["Alerts"])


@router.sub("", response_model=ttm.AlertPydantic)
async def sub_alerts(req: SubscriptionRequest):
    return alert_events.alerts.pipe(rxops.filter(lambda x: x is not None))


@router.get("", response_model=List[ttm.AlertPydantic])
async def get_alerts(repo: AlertRepository = Depends(alert_repo_dep)):
    return await repo.get_all_alerts()


@router.get("/{id}", response_model=ttm.AlertPydantic)
async def get_alert(id: str, repo: AlertRepository = Depends(alert_repo_dep)):
    alert = await repo.get_alert(id)
    if alert is None:
        raise HTTPException(404, f"Alert with ID {id} not found")
    return alert


@router.post("", status_code=201, response_model=ttm.AlertPydantic)
async def create_alert(
    id: str, category: str, repo: AlertRepository = Depends(alert_repo_dep)
):
    alert = await repo.create_alert(id, category)
    if alert is None:
        raise HTTPException(404, f"Could not create alert with ID {id}")
    return alert


@router.post("/{id}", status_code=201, response_model=ttm.AlertPydantic)
async def acknowledge_alert(id: str, repo: AlertRepository = Depends(alert_repo_dep)):
    alert = await repo.acknowledge_alert(id)
    if alert is None:
        raise HTTPException(404, f"Could acknowledge alert with ID {id}")
    return alert
