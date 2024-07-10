from .base import PydanticModel
from .tortoise_models.alerts import Alert as DbAlert


class Alert(PydanticModel):
    id: str
    original_id: str
    category: DbAlert.Category
    unix_millis_created_time: int
    acknowledged_by: str
    unix_millis_acknowledged_time: int
