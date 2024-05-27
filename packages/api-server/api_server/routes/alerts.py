from datetime import datetime
from typing import List

from fastapi import HTTPException
from rx import operators as rxops

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import rmf_gateway
from api_server.models import AlertRequest, AlertResponse
from api_server.models import tortoise_models as ttm
from api_server.rmf_io import alert_events

router = FastIORouter(tags=["Alerts"])


@router.sub("/requests", response_model=AlertRequest)
async def sub_alerts(_req: SubscriptionRequest):
    return alert_events.alert_requests.pipe(rxops.filter(lambda x: x is not None))


@router.post("/request", status_code=201, response_model=AlertRequest)
async def create_new_alert(alert: AlertRequest):
    """
    Creates a new alert.
    """
    exists = await ttm.AlertRequest.exists(id=alert.id)
    if exists:
        raise HTTPException(409, f"Alert with ID {alert.id} already exists")

    await ttm.AlertRequest.create(
        id=alert.id,
        data=alert.json(),
        response_expected=(len(alert.responses_available) > 0),
        task_id=alert.task_id,
    )

    if alert.display:
        alert_events.alert_requests.on_next(alert)
    return alert


@router.get("/request/{alert_id}", response_model=AlertRequest)
async def get_alert(alert_id: str):
    """
    Gets an alert based on the alert ID.
    """
    alert = await ttm.AlertRequest.get_or_none(id=alert_id)
    if alert is None:
        raise HTTPException(404, f"Alert with ID {alert_id} does not exists")

    alert_model = AlertRequest(**alert.data)
    return alert_model


@router.sub("/responses", response_model=AlertResponse)
async def sub_alert_responses(_req: SubscriptionRequest):
    return alert_events.alert_responses.pipe(rxops.filter(lambda x: x is not None))


@router.post(
    "/request/{alert_id}/respond", status_code=201, response_model=AlertResponse
)
async def respond_to_alert(alert_id: str, response: str):
    """
    Responds to an existing alert. The response must be one of the available
    responses listed in the alert.
    """
    alert = await ttm.AlertRequest.get_or_none(id=alert_id)
    if alert is None:
        raise HTTPException(404, f"Alert with ID {alert_id} does not exists")

    alert_model = AlertRequest(**alert.data)
    if response not in alert_model.responses_available:
        raise HTTPException(
            422,
            f"Alert with ID {alert_model.id} does not have allow response of {response}",
        )

    alert_response_model = AlertResponse(
        id=alert_id,
        unix_millis_response_time=round(datetime.now().timestamp() * 1000),
        response=response,
    )
    await ttm.AlertResponse.create(
        id=alert_id, alert_request=alert, data=alert_response_model.json()
    )
    alert_events.alert_responses.on_next(alert_response_model)

    rmf_gateway().respond_to_alert(alert_id, response)
    return alert_response_model


@router.get("/request/{alert_id}/response", response_model=AlertResponse)
async def get_alert_response(alert_id: str):
    """
    Gets the response to the alert based on the alert ID.
    """
    response = await ttm.AlertResponse.get_or_none(id=alert_id)
    if response is None:
        raise HTTPException(
            404, f"Response to alert with ID {alert_id} does not exists"
        )

    response_model = AlertResponse(**response.data)
    return response_model


@router.get("/requests/task/{task_id}", response_model=List[AlertRequest])
async def get_alerts_of_task(task_id: str, unresponded: bool = True):
    """
    Returns all the alerts associated to a task ID. Provides the option to only
    return alerts that have not been responded to yet.
    """
    if unresponded:
        task_id_alerts = await ttm.AlertRequest.filter(
            response_expected=True,
            task_id=task_id,
            alert_response=None,
        )
    else:
        task_id_alerts = await ttm.AlertRequest.filter(task_id=task_id)

    alert_models = [AlertRequest(**alert.data) for alert in task_id_alerts]
    return alert_models


@router.get("/unresponded_requests", response_model=List[AlertRequest])
async def get_unresponded_alerts():
    """
    Returns the list of alert IDs that have yet to be responded to, while a
    response was required.
    """
    unresponded_alerts = await ttm.AlertRequest.filter(
        alert_response=None, response_expected=True
    )
    return [AlertRequest(**alert.data) for alert in unresponded_alerts]
