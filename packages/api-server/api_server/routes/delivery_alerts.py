from datetime import datetime
from typing import List

from fastapi import HTTPException
from rx import operators as rxops

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import rmf_gateway
from api_server.models import tortoise_models as ttm
from api_server.models.delivery_alerts import (
    action_to_msg,
    category_to_msg,
    tier_to_msg,
)
from api_server.rmf_io import rmf_events

router = FastIORouter(tags=["DeliveryAlerts"])


@router.sub("", response_model=ttm.DeliveryAlertPydantic)
async def sub_delivery_alerts(_req: SubscriptionRequest):
    return rmf_events.delivery_alerts.pipe(rxops.filter(lambda x: x is not None))


@router.get("", response_model=List[ttm.DeliveryAlertPydantic])
async def get_delivery_alerts():
    delivery_alerts = await ttm.DeliveryAlert.all()
    return [
        await ttm.DeliveryAlertPydantic.from_tortoise_orm(a) for a in delivery_alerts
    ]


@router.get("/{delivery_alert_id}", response_model=ttm.DeliveryAlertPydantic)
async def get_delivery_alert(delivery_alert_id: str):
    delivery_alert = await ttm.DeliveryAlert.get_or_none(id=delivery_alert_id)
    if delivery_alert is None:
        raise HTTPException(
            404, f"Delivery alert with ID {delivery_alert_id} not found"
        )
    delivery_alert_pydantic = await ttm.DeliveryAlertPydantic.from_tortoise_orm(
        delivery_alert
    )
    return delivery_alert_pydantic


@router.post("", status_code=201, response_model=ttm.DeliveryAlertPydantic)
async def create_delivery_alert(category: str, tier: str, task_id: str, message: str):
    timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    delivery_alert_id = f"delivery-alert-{timestamp}"
    try:
        delivery_alert = await ttm.DeliveryAlert.create(
            id=delivery_alert_id,
            category=category,
            tier=tier,
            task_id=task_id,
            action="waiting",
            message=message,
        )
    except Exception as e:
        raise HTTPException(400, f"Could not create delivery alert: {e}") from e

    delivery_alert_pydantic = await ttm.DeliveryAlertPydantic.from_tortoise_orm(
        delivery_alert
    )
    rmf_events.delivery_alerts.on_next(delivery_alert_pydantic)
    return delivery_alert_pydantic


@router.post("/{delivery_alert_id}/action", response_model=ttm.DeliveryAlertPydantic)
async def update_delivery_alert_action(delivery_alert_id: str, action: str):
    delivery_alert = await ttm.DeliveryAlert.get_or_none(id=delivery_alert_id)
    if delivery_alert is None:
        raise HTTPException(
            404, f"Delivery alert with ID {delivery_alert_id} not found"
        )

    try:
        delivery_alert.update_from_dict({"action": action})
    except Exception as e:
        raise HTTPException(
            404,
            f"Failed to update delivery alert {delivery_alert_id} with action {action}: {e}",
        ) from e
    await delivery_alert.save()

    delivery_alert_pydantic = await ttm.DeliveryAlertPydantic.from_tortoise_orm(
        delivery_alert
    )

    rmf_gateway().respond_to_delivery_alert(
        id=delivery_alert_pydantic.id,
        category=category_to_msg(delivery_alert_pydantic.category),
        tier=tier_to_msg(delivery_alert_pydantic.tier),
        task_id=delivery_alert_pydantic.task_id,
        action=action_to_msg(delivery_alert_pydantic.action),
        message=delivery_alert_pydantic.message,
    )

    return delivery_alert_pydantic
