from typing import Generic, List, TypeVar

from pydantic import BaseModel

ItemT = TypeVar("ItemT")


class Pagination(Generic[ItemT], BaseModel):
    items: List[ItemT]

    @staticmethod
    def response_model(ResponseItemT):
        """
        Creates a response model that can be used in FastAPI to generate correct docs.
        In order to prevent class name conflicts, this should not be used directly,
        it should be used as a super class.
        """

        class ResponseModel(Pagination):
            items: List[ResponseItemT]

        return ResponseModel
