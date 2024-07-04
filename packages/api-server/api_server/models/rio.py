from typing import Any

from pydantic import BaseModel, ConfigDict


class Rio(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: str
    type: str
    data: dict[str, Any]
