from typing import Optional

from pydantic import BaseModel


class Delivery(BaseModel):
    id: str
    fleet_name: Optional[str]
    robot_name: Optional[str]
    destinations: Optional[list[str]]
