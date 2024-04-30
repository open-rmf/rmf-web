from typing import Dict, List

from pydantic import BaseModel


class TaskFavorite(BaseModel):
    id: str
    name: str
    unix_millis_earliest_start_time: int
    priority: Dict | None
    category: str
    description: Dict | None
    user: str
    labels: List[str] | None
