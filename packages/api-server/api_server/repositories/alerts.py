from datetime import datetime
from typing import List, Optional

from fastapi import Depends, HTTPException

from api_server.authenticator import user_dep
from api_server.logger import logger
from api_server.models import User
from api_server.models import tortoise_models as ttm


class AlertRepository:
    def __init__(self, user: User):
        self.user = user

    async def get_all_alerts(self) -> List[ttm.AlertPydantic]:
        alerts = await ttm.Alert.all()
        return [await ttm.AlertPydantic.from_tortoise_orm(a) for a in alerts]

    async def alert_exists(self, id: str) -> bool:
        result = await ttm.Alert.exists(id=id)
        return result

    async def get_alert(self, id: str) -> Optional[ttm.AlertPydantic]:
        alert = await ttm.Alert.get_or_none(id=id)
        if alert is None:
            logger.error(f"Alert with ID {id} not found")
            return None
        alert_pydantic = await ttm.AlertPydantic.from_tortoise_orm(alert)
        return alert_pydantic

    async def create_alert(self, id: str, category: str) -> Optional[ttm.AlertPydantic]:
        alert, _ = await ttm.Alert.update_or_create(
            {
                "original_id": id,
                "category": category,
                "unix_millis_created_time": round(datetime.now().timestamp() * 1e3),
                "acknowledged_by": None,
                "unix_millis_acknowledged_time": None,
            },
            id=id,
        )
        if alert is None:
            logger.error(f"Failed to create Alert with ID {id}")
            return None
        alert_pydantic = await ttm.AlertPydantic.from_tortoise_orm(alert)
        return alert_pydantic

    async def acknowledge_alert(self, id: str) -> Optional[ttm.AlertPydantic]:
        alert = await ttm.Alert.get_or_none(id=id)
        if alert is None:
            acknowledged_alert = await ttm.Alert.filter(original_id=id).first()
            if acknowledged_alert is None:
                logger.error(f"No existing or past alert with ID {id} found.")
                return None
            acknowledged_alert_pydantic = await ttm.AlertPydantic.from_tortoise_orm(
                acknowledged_alert
            )
            return acknowledged_alert_pydantic

        ack_time = datetime.now()
        epoch = datetime.utcfromtimestamp(0)
        ack_unix_millis = round((ack_time - epoch).total_seconds() * 1000)
        new_id = f"{id}__{ack_unix_millis}"

        ack_alert = alert.clone(pk=new_id)
        # TODO(aaronchongth): remove the following line once we bump
        # tortoise-orm to include
        # https://github.com/tortoise/tortoise-orm/pull/1131. This is a
        # temporary workaround.
        ack_alert._custom_generated_pk = True
        ack_alert.update_from_dict(
            {
                "acknowledged_by": self.user.username,
                "unix_millis_acknowledged_time": round(ack_time.timestamp() * 1e3),
            }
        )
        await ack_alert.save()
        await alert.delete()
        ack_alert_pydantic = await ttm.AlertPydantic.from_tortoise_orm(ack_alert)
        return ack_alert_pydantic


def alert_repo_dep(user: User = Depends(user_dep)):
    return AlertRepository(user)
