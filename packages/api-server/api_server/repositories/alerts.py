import logging
from collections import deque
from datetime import datetime
from typing import List, Optional

from api_server.models import AlertRequest, AlertResponse
from api_server.models import tortoise_models as ttm

# TODO: not hardcode all these expected values
LocationAlertSuccessResponse = "success"
LocationAlertFailResponse = "fail"
LocationAlertTypeParameterName = "type"
LocationAlertTypeParameterValue = "location_result"
LocationAlertLocationParameterName = "location_name"
LocationAlertFinalCheckTypeParameterValue = "check_all_task_location_alerts"


def get_location_from_location_alert(alert: AlertRequest) -> Optional[str]:
    """
    Returns the location name from a location alert when possible, otherwise
    returns None.
    Note: This is an experimental feature and may be subjected to
    modifications often.
    """
    if (
        len(alert.alert_parameters) < 2
        or LocationAlertSuccessResponse not in alert.responses_available
        or LocationAlertFailResponse not in alert.responses_available
    ):
        return None

    # Check type
    alert_type = None
    for param in alert.alert_parameters:
        if param.name == LocationAlertTypeParameterName:
            alert_type = param.value
            break
    if alert_type != LocationAlertTypeParameterValue:
        return None

    # Check location name
    # TODO: make sure that there are no duplicated locations that have
    # not been responded to yet
    for param in alert.alert_parameters:
        if param.name == LocationAlertLocationParameterName:
            return param.value
    return None


def is_final_location_alert_check(alert: AlertRequest) -> bool:
    """
    Checks if the alert request requires a check on all location alerts of this
    task.
    Note: This is an experimental feature and may be subjected to
    modifications often.
    """
    if (
        alert.task_id is None
        or len(alert.alert_parameters) < 1
        or LocationAlertSuccessResponse not in alert.responses_available
        or LocationAlertFailResponse not in alert.responses_available
    ):
        return False

    # Check type
    for param in alert.alert_parameters:
        if param.name == LocationAlertTypeParameterName:
            if param.value == LocationAlertFinalCheckTypeParameterValue:
                return True
            return False
    return False


class LRUCache:
    def __init__(self, capacity: int):
        self._cache = deque(maxlen=capacity)
        self._lookup = {}

    def add(self, key, value):
        if key in self._lookup:
            self._cache.remove(key)
        elif len(self._cache) == self._cache.maxlen:
            oldest_key = self._cache.popleft()
            del self._lookup[oldest_key]

        self._cache.append(key)
        self._lookup[key] = value

    def remove(self, key):
        if key in self._lookup:
            self._cache.remove(key)
            del self._lookup[key]

    def lookup(self, key):
        if key in self._lookup:
            self._cache.remove(key)
            self._cache.append(key)
            return self._lookup[key]
        return None


task_id_to_all_locations_success_cache: LRUCache = LRUCache(20)


class AlertRepository:
    async def create_new_alert(self, alert: AlertRequest) -> Optional[AlertRequest]:
        exists = await ttm.AlertRequest.exists(id=alert.id)
        if exists:
            logging.error(f"Alert with ID {alert.id} already exists")
            return None

        await ttm.AlertRequest.create(
            id=alert.id,
            data=alert.json(),
            response_expected=(len(alert.responses_available) > 0),
            task_id=alert.task_id,
        )
        return alert

    async def get_alert(self, alert_id: str) -> Optional[AlertRequest]:
        alert = await ttm.AlertRequest.get_or_none(id=alert_id)
        if alert is None:
            logging.error(f"Alert with ID {alert_id} does not exists")
            return None

        alert_model = AlertRequest.from_tortoise(alert)
        return alert_model

    async def create_response(
        self, alert_id: str, response: str
    ) -> Optional[AlertResponse]:
        alert = await ttm.AlertRequest.get_or_none(id=alert_id)
        if alert is None:
            logging.error(f"Alert with ID {alert_id} does not exists")
            return None

        alert_model = AlertRequest.from_tortoise(alert)
        if response not in alert_model.responses_available:
            logging.error(
                f"Alert with ID {alert_model.id} does not have allow response of {response}"
            )
            return None

        alert_response_model = AlertResponse(
            id=alert_id,
            unix_millis_response_time=round(datetime.now().timestamp() * 1000),
            response=response,
        )
        await ttm.AlertResponse.create(
            id=alert_id, alert_request=alert, data=alert_response_model.json()
        )
        return alert_response_model

    async def get_alert_response(self, alert_id: str) -> Optional[AlertResponse]:
        response = await ttm.AlertResponse.get_or_none(id=alert_id)
        if response is None:
            logging.error(f"Response to alert with ID {alert_id} does not exists")
            return None

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

    async def get_unresponded_alerts(self) -> List[AlertRequest]:
        unresponded_alerts = await ttm.AlertRequest.filter(
            alert_response=None, response_expected=True
        )
        return [AlertRequest.from_tortoise(alert) for alert in unresponded_alerts]

    async def create_location_alert_response(
        self,
        task_id: str,
        location: str,
        success: bool,
    ) -> Optional[AlertResponse]:
        """
        Creates an alert response for a location alert of the task.
        Note: This is an experimental feature and may be subjected to
        modifications often.
        """
        alerts = await self.get_alerts_of_task(task_id=task_id, unresponded=True)
        if len(alerts) == 0:
            logging.error(
                f"There are no location alerts awaiting response for task {task_id}"
            )
            return None

        for alert in alerts:
            location_alert_location = get_location_from_location_alert(alert)
            if location_alert_location is None:
                continue

            if location_alert_location == location:
                response = (
                    LocationAlertSuccessResponse
                    if success
                    else LocationAlertFailResponse
                )
                alert_response_model = await self.create_response(alert.id, response)
                if alert_response_model is None:
                    logging.error(
                        f"Failed to create response {response} to alert with ID {alert.id}"
                    )
                    return None

                # Cache if all locations of this task has been successful so far
                cache = task_id_to_all_locations_success_cache.lookup(task_id)
                if cache is None:
                    task_id_to_all_locations_success_cache.add(task_id, success)
                else:
                    task_id_to_all_locations_success_cache.add(
                        task_id, cache and success
                    )

                return alert_response_model

        logging.error(
            f"Task {task_id} is not awaiting completion of location {location}"
        )
        return None
