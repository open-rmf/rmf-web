from typing import Generic, List, TypeVar

from pydantic import BaseModel

ItemT = TypeVar("ItemT")


class Pagination(Generic[ItemT], BaseModel):
    items: List[ItemT]
    total_count: int

    @classmethod
    def response_model(cls, ResponseItemT):
        """
        Creates a response model that can be used in FastAPI to generate correct docs.
        """

        class ResponseModel(cls):
            items: List[ResponseItemT]

        return ResponseModel
