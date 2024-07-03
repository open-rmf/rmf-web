from pydantic import BaseModel, ConfigDict

from api_server.models.tortoise_models.alerts import Alert as DbAlert


class Alert(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: str
    original_id: str
    category: DbAlert.Category
    unix_millis_created_time: int
    acknowledged_by: str
    unix_millis_acknowledged_time: int
