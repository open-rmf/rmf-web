from typing import Any

from pydantic import BaseModel


class Rio(BaseModel):
    id: str
    type: str
    data: dict[str, Any]
