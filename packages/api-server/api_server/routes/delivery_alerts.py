# pyright: reportGeneralTypeIssues=false

from datetime import datetime
from typing import List, Optional

from fastapi import HTTPException, Query
from rx import operators as rxops
from tortoise.exceptions import FieldError

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import rmf_gateway
from api_server.logger import logger
from api_server.models import FleetState
from api_server.models import tortoise_models as ttm
from api_server.models.delivery_alerts import (
    DeliveryAlert,
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


@router.get("/query", response_model=List[ttm.DeliveryAlertPydantic])
async def query_delivery_alerts(
    category: Optional[str] = Query(
        None, description="comma separated list of alert categories"
    ),
    tier: Optional[str] = Query(None, description="comma separated of tier"),
    task_id: Optional[str] = Query(None, description="comma separated list of task_id"),
    action: Optional[str] = Query(None, description="comma separated list of action"),
    message: Optional[str] = Query(None, description="comma separated of message"),
):
    filters = {}
    if category is not None:
        filters["category__in"] = category.split(",")
    if tier is not None:
        filters["tier__in"] = tier.split(",")
    if task_id is not None:
        filters["task_id__in"] = task_id.split(",")
    if action is not None:
        filters["action__in"] = action.split(",")
    if message is not None:
        filters["message__in"] = message.split(",")

    try:
        delivery_alerts = await ttm.DeliveryAlert.filter(**filters)
        delivery_alerts_pydantic = [
            await ttm.DeliveryAlertPydantic.from_tortoise_orm(a)
            for a in delivery_alerts
        ]
    except FieldError as e:
        raise HTTPException(422, str(e)) from e

    ongoing_delivery_alerts = []
    # Check for dangling delivery alerts, where none of the fleets are still
    # performing the task
    current_task_ids = []
    try:
        db_states = await ttm.FleetState.all().values_list("data", flat=True)
        fleets = [FleetState(**s) for s in db_states]
        for f in fleets:
            for robot in f.robots.values():
                if robot.task_id is not None and len(robot.task_id) != 0:
                    current_task_ids.append(robot.task_id)
    except Exception as e:  # pylint: disable=broad-except
        logger.error(f"Failed to retrieve current running task IDs: {e}")
        return []

    logger.info(f"Current running task IDs: {current_task_ids}")
    for d in delivery_alerts_pydantic:
        if (
            d.task_id is not None
            and len(d.task_id) != 0
            and d.task_id not in current_task_ids
        ):
            logger.info(f"Found a dangling alert: {d}")
            try:
                dangling_alert = await ttm.DeliveryAlert.get_or_none(id=d.id)
                dangling_alert.update_from_dict({"action": "resume"})
                await dangling_alert.save()
            except Exception as e:  # pylint: disable=broad-except
                logger.error(f"Failed to resolve dangling alert ID {d.id}: {e}")
                continue
            logger.info(
                f"Resolved dangling alert ID {d.id} by updating action to resume"
            )
        else:
            ongoing_delivery_alerts.append(d)
    return ongoing_delivery_alerts


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
    # Fill up the model, let the book keeper handle saving and cancelling old
    # delivery alerts
    timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    delivery_alert_id = f"delivery-alert-{timestamp}"
    delivery_alert_pydantic = DeliveryAlert(
        id=delivery_alert_id,
        category=category,
        tier=tier,
        task_id=task_id,
        action="waiting",
        message=message,
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

    delivery_alert_pydantic = DeliveryAlert.from_tortoise(delivery_alert)

    rmf_gateway().respond_to_delivery_alert(
        alert_id=delivery_alert_pydantic.id,
        category=category_to_msg(delivery_alert_pydantic.category),
        tier=tier_to_msg(delivery_alert_pydantic.tier),
        task_id=delivery_alert_pydantic.task_id,
        action=action_to_msg(delivery_alert_pydantic.action),
        message=delivery_alert_pydantic.message,
    )

    rmf_events.delivery_alerts.on_next(delivery_alert_pydantic)
    return delivery_alert_pydantic
