import logging
from datetime import datetime

from fastapi import Depends

from api_server.authenticator import user_dep
from api_server.models import Alert, User
from api_server.models import tortoise_models as ttm
from api_server.repositories.tasks import TaskRepository


class AlertRepository:
    def __init__(
        self,
        user: User = Depends(user_dep),
        task_repo: TaskRepository = Depends(TaskRepository),
    ):
        self.user = user
        self.task_repo = task_repo

    async def get_all_alerts(self) -> list[Alert]:
        alerts = await ttm.Alert.all()
        return [Alert.model_validate(a) for a in alerts]

    async def alert_exists(self, alert_id: str) -> bool:
        result = await ttm.Alert.exists(id=alert_id)
        return result

    async def alert_original_id_exists(self, original_id: str) -> bool:
        result = await ttm.Alert.exists(original_id=original_id)
        return result

    async def get_alert(self, alert_id: str) -> Alert | None:
        alert = await ttm.Alert.get_or_none(id=alert_id)
        if alert is None:
            logging.error(f"Alert with ID {alert_id} not found")
            return None
        alert_pydantic = Alert.model_validate(alert)
        return alert_pydantic

    async def create_alert(self, alert_id: str, category: str) -> Alert | None:
        alert, _ = await ttm.Alert.update_or_create(
            {
                "original_id": alert_id,
                "category": category,
                "unix_millis_created_time": round(datetime.now().timestamp() * 1e3),
                "acknowledged_by": None,
                "unix_millis_acknowledged_time": None,
            },
            id=alert_id,
        )
        if alert is None:
            logging.error(f"Failed to create Alert with ID {alert_id}")
            return None
        alert_pydantic = Alert.model_validate(alert)
        return alert_pydantic

    async def acknowledge_alert(self, alert_id: str) -> Alert | None:
        alert = await ttm.Alert.get_or_none(id=alert_id)
        if alert is None:
            acknowledged_alert = await ttm.Alert.filter(original_id=alert_id).first()
            if acknowledged_alert is None:
                logging.error(f"No existing or past alert with ID {alert_id} found.")
                return None
            acknowledged_alert_pydantic = Alert.model_validate(acknowledged_alert)
            return acknowledged_alert_pydantic

        ack_time = datetime.now()
        epoch = datetime.fromtimestamp(0)
        ack_unix_millis = round((ack_time - epoch).total_seconds() * 1000)
        new_id = f"{alert_id}__{ack_unix_millis}"

        ack_alert = alert.clone(pk=new_id)
        # TODO(aaronchongth): remove the following line once we bump
        # tortoise-orm to include
        # https://github.com/tortoise/tortoise-orm/pull/1131. This is a
        # temporary workaround.
        ack_alert._custom_generated_pk = True  # pylint: disable=W0212
        unix_millis_acknowledged_time = round(ack_time.timestamp() * 1e3)
        ack_alert.update_from_dict(
            {
                "acknowledged_by": self.user.username,
                "unix_millis_acknowledged_time": unix_millis_acknowledged_time,
            }
        )
        await ack_alert.save()
        await alert.delete()
        ack_alert_pydantic = Alert.model_validate(ack_alert)
        return ack_alert_pydantic
