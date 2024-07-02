from fastapi import Depends
from reactivex import operators as rxops

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import rmf_gateway
from api_server.logging import LoggerAdapter, get_logger
from api_server.models.delivery_alerts import DeliveryAlert
from api_server.rmf_io import rmf_events

router = FastIORouter(tags=["DeliveryAlerts"])


@router.sub("", response_model=DeliveryAlert)
async def sub_delivery_alerts(_req: SubscriptionRequest):
    return rmf_events.delivery_alerts.pipe(rxops.filter(lambda x: x is not None))


@router.post("/{delivery_alert_id}/response", response_model=DeliveryAlert)
async def respond_to_delivery_alert(
    delivery_alert_id: str,
    category: DeliveryAlert.Category,
    tier: DeliveryAlert.Tier,
    task_id: str,
    action: DeliveryAlert.Action,
    message: str,
    logger: LoggerAdapter = Depends(get_logger),
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
    rmf_gateway().respond_to_delivery_alert(
        alert_id=delivery_alert.id,
        category=delivery_alert.category.value,
        tier=delivery_alert.tier.value,
        task_id=delivery_alert.task_id,
        action=delivery_alert.action.value,
        message=delivery_alert.message,
    )
    rmf_events.delivery_alerts.on_next(delivery_alert)
