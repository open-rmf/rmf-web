from datetime import datetime
from typing import List

from fastapi import Depends, HTTPException
from rx import operators as rxops

from api_server.authenticator import user_dep
from api_server.dependencies import sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import User
from api_server.models import tortoise_models as ttm
from api_server.rmf_io import alert_events

router = FastIORouter(tags=["Alerts"])


@router.sub("", response_model=ttm.AlertPydantic)
async def sub_alerts(req: SubscriptionRequest):
    return alert_events.alerts.pipe(rxops.filter(lambda x: x is not None))


@router.get("", response_model=List[ttm.AlertPydantic])
async def get_alerts():
    alerts = await ttm.Alert.all()
    return alerts


@router.get("/{id}", response_model=ttm.AlertPydantic)
async def get_alert(id: str):
    alert = await ttm.Alert.get_or_none(id=id)
    if alert is None:
        raise HTTPException(404, f"Alert with ID {id} not found")
    return alert


@router.post("", status_code=201, response_model=ttm.AlertPydantic)
async def create_alert(id: str, category: str):
    alert, _ = await ttm.Alert.update_or_create(
        {
            "category": category,
            "created_on": datetime.now(),
            "acknowledged_by": None,
            "acknowledged_on": None,
        },
        id=id,
    )
    return alert


@router.post("/{id}", status_code=201, response_model=ttm.AlertPydantic)
async def acknowledge_alert(id: str, user: User = Depends(user_dep)):
    alert = await ttm.Alert.get_or_none(id=id)
    if alert is not None:
        alert.update_from_dict(
            {
                "acknowledged_by": user.username,
                "acknowledged_on": datetime.now(),
            }
        )
        await alert.save()
    else:
        raise HTTPException(404, f"Alert with ID {id} not found.")
    return alert
