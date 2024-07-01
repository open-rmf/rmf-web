from enum import Enum
from typing import List, Optional

from pydantic import BaseModel

from . import tortoise_models as ttm


class AlertParameter(BaseModel):
    name: str
    value: str


class AlertResponse(BaseModel):
    id: str
    unix_millis_response_time: int
    response: str

    @staticmethod
    def from_tortoise(tortoise: ttm.AlertResponse) -> "AlertResponse":
        return AlertResponse(**tortoise.data)

    async def save(self) -> None:
        await ttm.AlertResponse.update_or_create(
            {
                "data": self.json(),
            },
            id=self.id,
        )


class AlertRequest(BaseModel):
    class Tier(str, Enum):
        Info = "info"
        Warning = "warning"
        Error = "error"

    id: str
    unix_millis_alert_time: int
    title: str
    subtitle: str
    message: str
    display: bool
    tier: Tier
    responses_available: List[str]
    alert_parameters: List[AlertParameter]
    task_id: Optional[str]

    @staticmethod
    def from_tortoise(tortoise: ttm.AlertRequest) -> "AlertRequest":
        return AlertRequest(**tortoise.data)

    async def save(self) -> None:
        await ttm.AlertRequest.update_or_create(
            {
                "data": self.json(),
                "response_expected": (len(self.responses_available) > 0),
                "task_id": self.task_id,
            },
            id=self.id,
        )
