from typing import List

from fastapi import Depends, HTTPException
from rx import operators as rxops

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import rmf_gateway
from api_server.models import AlertRequest, AlertResponse
from api_server.repositories import AlertRepository
from api_server.rmf_io import alert_events

router = FastIORouter(tags=["Alerts"])


@router.sub("/requests", response_model=AlertRequest)
async def sub_alerts(_req: SubscriptionRequest):
    return alert_events.alert_requests.pipe(rxops.filter(lambda x: x is not None))


@router.post("/request", status_code=201, response_model=AlertRequest)
async def create_new_alert(
    alert: AlertRequest, repo: AlertRepository = Depends(AlertRepository)
):
    """
    Creates a new alert.
    """
    created_alert = await repo.create_new_alert(alert)
    if created_alert is None:
        raise HTTPException(409, f"Failed to create alert with ID {alert.id}")

    if created_alert.display:
        alert_events.alert_requests.on_next(created_alert)
    return created_alert


@router.get("/request/{alert_id}", response_model=AlertRequest)
async def get_alert(alert_id: str, repo: AlertRepository = Depends(AlertRepository)):
    """
    Gets an alert based on the alert ID.
    """
    alert_model = await repo.get_alert(alert_id)
    if alert_model is None:
        raise HTTPException(404, f"Alert with ID {alert_id} does not exists")

    return alert_model


@router.sub("/responses", response_model=AlertResponse)
async def sub_alert_responses(_req: SubscriptionRequest):
    return alert_events.alert_responses.pipe(rxops.filter(lambda x: x is not None))


@router.post(
    "/request/{alert_id}/respond", status_code=201, response_model=AlertResponse
)
async def respond_to_alert(
    alert_id: str, response: str, repo: AlertRepository = Depends(AlertRepository)
):
    """
    Responds to an existing alert. The response must be one of the available
    responses listed in the alert.
    """
    alert_response_model = await repo.create_response(alert_id, response)
    if alert_response_model is None:
        raise HTTPException(
            422,
            f"Failed to create response {response} to alert with ID {alert_id}",
        )

    alert_events.alert_responses.on_next(alert_response_model)
    rmf_gateway().respond_to_alert(alert_id, response)
    return alert_response_model


@router.get("/request/{alert_id}/response", response_model=AlertResponse)
async def get_alert_response(
    alert_id: str, repo: AlertRepository = Depends(AlertRepository)
):
    """
    Gets the response to the alert based on the alert ID.
    """
    response_model = await repo.get_alert_response(alert_id)
    if response_model is None:
        raise HTTPException(
            404, f"Response to alert with ID {alert_id} does not exists"
        )

    return response_model


@router.get("/requests/task/{task_id}", response_model=List[AlertRequest])
async def get_alerts_of_task(
    task_id: str,
    unresponded: bool = True,
    repo: AlertRepository = Depends(AlertRepository),
):
    """
    Returns all the alerts associated to a task ID. Provides the option to only
    return alerts that have not been responded to yet.
    """
    return await repo.get_alerts_of_task(task_id, unresponded)


@router.get("/unresponded_requests", response_model=List[AlertRequest])
async def get_unresponded_alerts(repo: AlertRepository = Depends(AlertRepository)):
    """
    Returns the list of alert IDs that have yet to be responded to, while a
    response was required.
    """
    return await repo.get_unresponded_alerts()
