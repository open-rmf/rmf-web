from datetime import datetime
from typing import List

from fastapi import Depends, HTTPException

from api_server.authenticator import user_dep
from api_server.fast_io import FastIORouter
from api_server.models import User
from api_server.models import tortoise_models as ttm

router = FastIORouter(tags=["Alerts"])


@router.get("", response_model=list[ttm.AlertPydantic])
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
async def create_alert(
    id: str,
    details: ttm.AlertDetailsPydantic,
):
    alert, created = await ttm.Alert.get_or_create(
        {
            "created_on": datetime.now(),
            "detail": ttm.AlertDetails(**details.dict()),
            "acknowledged_by": None,
            "acknowledged_on": None,
        },
        id=id,
    )
    if created:
        return alert
    else:
        raise HTTPException(409, f"Duplicated alert ID {id} found.")


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
