from typing import Annotated, List, Optional

from fastapi import Depends, HTTPException
from tortoise.exceptions import IntegrityError

from api_server.exceptions import AlreadyExistsError, InvalidInputError, NotFoundError
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import RmfGateway, get_rmf_gateway
from api_server.models import AlertRequest, AlertResponse, Pagination
from api_server.repositories import AlertRepository
from api_server.rmf_io import AlertEvents, get_alert_events

router = FastIORouter(tags=["Alerts"])


@router.sub("/requests", response_model=AlertRequest)
async def sub_alerts(_req: SubscriptionRequest):
    alert_events = get_alert_events()
    return alert_events.alert_requests


@router.post("/request", status_code=201, response_model=AlertRequest)
async def create_new_alert(
    alert: AlertRequest,
    repo: Annotated[AlertRepository, Depends(AlertRepository)],
    alert_events: Annotated[AlertEvents, Depends(get_alert_events)],
):
    """
    Creates a new alert.
    """
    try:
        created_alert = await repo.create_new_alert(alert)
    except IntegrityError as e:
        raise HTTPException(400, e) from e
    except AlreadyExistsError as e:
        raise HTTPException(409, str(e)) from e

    alert_events.alert_requests.on_next(created_alert)
    return created_alert


@router.get("/request/{alert_id}", response_model=AlertRequest)
async def get_alert(
    alert_id: str,
    repo: Annotated[AlertRepository, Depends(AlertRepository)],
):
    """
    Gets an alert based on the alert ID.
    """
    try:
        alert_model = await repo.get_alert(alert_id)
    except NotFoundError as e:
        raise HTTPException(404, str(e)) from e

    return alert_model


@router.sub("/responses", response_model=AlertResponse)
async def sub_alert_responses(_req: SubscriptionRequest):
    alert_events = get_alert_events()
    return alert_events.alert_responses


@router.post(
    "/request/{alert_id}/respond", status_code=201, response_model=AlertResponse
)
async def respond_to_alert(
    alert_id: str,
    response: str,
    repo: Annotated[AlertRepository, Depends(AlertRepository)],
    alert_events: Annotated[AlertEvents, Depends(get_alert_events)],
    rmf_gateway: Annotated[RmfGateway, Depends(get_rmf_gateway)],
):
    """
    Responds to an existing alert. The response must be one of the available
    responses listed in the alert.
    """
    try:
        alert_response_model = await repo.create_response(alert_id, response)
    except IntegrityError as e:
        raise HTTPException(400, e) from e
    except AlreadyExistsError as e:
        raise HTTPException(409, str(e)) from e
    except NotFoundError as e:
        raise HTTPException(404, str(e)) from e
    except InvalidInputError as e:
        raise HTTPException(400, str(e)) from e

    alert_events.alert_responses.on_next(alert_response_model)
    rmf_gateway.respond_to_alert(alert_id, response)
    return alert_response_model


@router.get("/request/{alert_id}/response", response_model=AlertResponse)
async def get_alert_response(
    alert_id: str,
    repo: Annotated[AlertRepository, Depends(AlertRepository)],
):
    """
    Gets the response to the alert based on the alert ID.
    """
    try:
        response_model = await repo.get_alert_response(alert_id)
    except NotFoundError as e:
        raise HTTPException(404, str(e)) from e

    return response_model


@router.get("/requests/task/{task_id}", response_model=List[AlertRequest])
async def get_alerts_of_task(
    task_id: str,
    repo: Annotated[AlertRepository, Depends(AlertRepository)],
    unresponded: bool = True,
):
    """
    Returns all the alerts associated to a task ID. Provides the option to only
    return alerts that have not been responded to yet.
    """
    return await repo.get_alerts_of_task(task_id, unresponded)


@router.get("/unresponded_requests", response_model=List[AlertRequest])
async def get_unresponded_alerts(
    repo: Annotated[AlertRepository, Depends(AlertRepository)],
    pagination: Optional[Pagination] = None,
):
    """
    Returns the list of alert IDs that have yet to be responded to, while a
    response was required.
    """
    return await repo.get_unresponded_alerts(pagination)
