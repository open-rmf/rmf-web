from datetime import datetime
from typing import List

from api_server.exceptions import AlreadyExistsError, InvalidInputError, NotFoundError
from api_server.models import AlertRequest, AlertResponse, Pagination
from api_server.models import tortoise_models as ttm


class AlertRepository:
    async def create_new_alert(self, alert: AlertRequest) -> AlertRequest:
        exists = await ttm.AlertRequest.exists(id=alert.id)
        if exists:
            raise AlreadyExistsError(f"Alert with ID {alert.id} already exists")

        await ttm.AlertRequest.create(
            id=alert.id,
            request_time=datetime.fromtimestamp(alert.unix_millis_alert_time / 1000),
            response_expected=(len(alert.responses_available) > 0),
            task_id=alert.task_id,
            data=alert.model_dump(),
        )
        return alert

    async def get_alert(self, alert_id: str) -> AlertRequest:
        alert = await ttm.AlertRequest.get_or_none(id=alert_id)
        if alert is None:
            raise NotFoundError(f"Alert with ID {alert_id} does not exists")

        alert_model = AlertRequest.from_tortoise(alert)
        return alert_model

    async def create_response(self, alert_id: str, response: str) -> AlertResponse:
        existing_response = await ttm.AlertResponse.get_or_none(id=alert_id)
        if existing_response is not None:
            existing_response_model = AlertResponse.from_tortoise(existing_response)
            raise AlreadyExistsError(
                f"Alert with ID {alert_id} already has a response of {existing_response_model.response}"
            )

        alert = await ttm.AlertRequest.get_or_none(id=alert_id)
        if alert is None:
            raise NotFoundError(f"Alert with ID {alert_id} does not exists")

        alert_model = AlertRequest.from_tortoise(alert)
        if response not in alert_model.responses_available:
            raise InvalidInputError(
                f"Response [{response}] is not a response option of alert with ID {alert_model.id}"
            )

        alert_response_model = AlertResponse(
            id=alert_id,
            unix_millis_response_time=round(datetime.now().timestamp() * 1000),
            response=response,
        )
        await ttm.AlertResponse.create(
            id=alert_id,
            response_time=datetime.fromtimestamp(
                alert_response_model.unix_millis_response_time / 1000
            ),
            response=response,
            data=alert_response_model.model_dump(),
            alert_request=alert,
        )
        return alert_response_model

    async def get_alert_response(self, alert_id: str) -> AlertResponse:
        response = await ttm.AlertResponse.get_or_none(id=alert_id)
        if response is None:
            raise NotFoundError(f"Response to alert with ID {alert_id} does not exists")

        response_model = AlertResponse.from_tortoise(response)
        return response_model

    async def get_alerts_of_task(
        self, task_id: str, unresponded: bool = True
    ) -> List[AlertRequest]:
        if unresponded:
            task_id_alerts = await ttm.AlertRequest.filter(
                response_expected=True,
                task_id=task_id,
                alert_response=None,
            )
        else:
            task_id_alerts = await ttm.AlertRequest.filter(task_id=task_id)

        alert_models = [AlertRequest.from_tortoise(alert) for alert in task_id_alerts]
        return alert_models

    async def get_unresponded_alerts(
        self, pagination: Pagination | None
    ) -> List[AlertRequest]:
        query = ttm.AlertRequest.filter(alert_response=None, response_expected=True)
        if pagination:
            query = query.limit(pagination.limit).offset(pagination.offset)
        unresponded_alerts = await query.all()
        return [AlertRequest.from_tortoise(alert) for alert in unresponded_alerts]
