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
            "original_id": id,
            "category": category,
            "unix_millis_created_time": round(datetime.now().timestamp() * 1e3),
            "acknowledged_by": None,
            "unix_millis_acknowledged_time": None,
        },
        id=id,
    )
    return alert


@router.post("/{id}", status_code=201, response_model=ttm.AlertPydantic)
async def acknowledge_alert(id: str, user: User = Depends(user_dep)):
    alert = await ttm.Alert.get_or_none(id=id)
    if alert is None:
        acknowledged_alert = await ttm.Alert.filter(original_id=id).first()
        if acknowledged_alert is None:
            raise HTTPException(404, f"No existing or past alert with ID {id} found.")
        else:
            return acknowledged_alert

    ack_time = datetime.now()
    epoch = datetime.utcfromtimestamp(0)
    ack_unix_millis = round((ack_time - epoch).total_seconds() * 1000)
    new_id = f"{id}__{ack_unix_millis}"

    ack_alert = alert.clone(pk=new_id)
    # TODO(aaronchongth): remove the following line once we bump tortoise-orm to
    # include https://github.com/tortoise/tortoise-orm/pull/1131. This is a
    # temporary workaround.
    ack_alert._custom_generated_pk = True
    ack_alert.update_from_dict(
        {
            "acknowledged_by": user.username,
            "unix_millis_acknowledged_time": round(ack_time.timestamp() * 1e3),
        }
    )
    await ack_alert.save()
    await alert.delete()
    return ack_alert
