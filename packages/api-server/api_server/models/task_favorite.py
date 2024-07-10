from .base import PydanticModel


class TaskFavorite(PydanticModel):
    id: str
    name: str
    unix_millis_earliest_start_time: int
    priority: dict | None
    category: str
    description: dict | None
    user: str
    task_definition_id: str
