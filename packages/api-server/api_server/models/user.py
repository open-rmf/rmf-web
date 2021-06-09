from typing import Set

from pydantic import BaseModel


class User(BaseModel):
    username: str
    roles: Set[str]
