from typing import Optional

from pydantic import BaseModel


class Pagination(BaseModel):
    limit: int
    offset: int
    order_by: Optional[str]
