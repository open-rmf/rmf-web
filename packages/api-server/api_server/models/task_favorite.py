from pydantic import BaseModel, ConfigDict


class TaskFavorite(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: str
    name: str
    unix_millis_earliest_start_time: int
    priority: dict | None
    category: str
    description: dict | None
    user: str
    task_definition_id: str
