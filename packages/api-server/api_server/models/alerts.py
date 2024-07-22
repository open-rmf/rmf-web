from enum import Enum

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
        return AlertResponse(**dict(tortoise.data))


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
    responses_available: list[str]
    alert_parameters: list[AlertParameter]
    task_id: str | None

    @staticmethod
    def from_tortoise(tortoise: ttm.AlertRequest) -> "AlertRequest":
        return AlertRequest(**dict(tortoise.data))
