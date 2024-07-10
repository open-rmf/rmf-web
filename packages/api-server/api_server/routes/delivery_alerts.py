from typing import Annotated

from fastapi import Depends
from reactivex import operators as rxops

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import RmfGateway, get_rmf_gateway
from api_server.logging import LoggerAdapter, get_logger
from api_server.models.delivery_alerts import DeliveryAlert
from api_server.rmf_io import RmfEvents, get_rmf_events

router = FastIORouter(tags=["DeliveryAlerts"])


@router.sub("", response_model=DeliveryAlert)
async def sub_delivery_alerts(_req: SubscriptionRequest):
    return get_rmf_events().delivery_alerts.pipe(rxops.filter(lambda x: x is not None))


@router.post("/{delivery_alert_id}/response", response_model=DeliveryAlert)
async def respond_to_delivery_alert(
    delivery_alert_id: str,
    category: DeliveryAlert.Category,
    tier: DeliveryAlert.Tier,
    task_id: str,
    action: DeliveryAlert.Action,
    message: str,
    logger: Annotated[LoggerAdapter, Depends(get_logger)],
    rmf_events: Annotated[RmfEvents, Depends(get_rmf_events)],
    rmf_gateway: Annotated[RmfGateway, Depends(get_rmf_gateway)],
):
    delivery_alert = DeliveryAlert(
        id=delivery_alert_id,
        category=category,
        tier=tier,
        action=action,
        task_id=task_id,
        message=message,
    )
    logger.info(delivery_alert)
    rmf_gateway.respond_to_delivery_alert(
        alert_id=delivery_alert.id,
        category=delivery_alert.category.to_rmf_value(),
        tier=delivery_alert.tier.to_rmf_value(),
        task_id=delivery_alert.task_id,
        action=delivery_alert.action.to_rmf_value(),
        message=delivery_alert.message,
    )
    rmf_events.delivery_alerts.on_next(delivery_alert)
