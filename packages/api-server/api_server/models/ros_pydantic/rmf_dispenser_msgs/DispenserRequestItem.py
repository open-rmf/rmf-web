# This is a generated file, do not edit

from typing import List

import pydantic


class DispenserRequestItem(pydantic.BaseModel):
    type_guid: str
    quantity: pydantic.conint(ge=-2147483648, le=2147483647)
    compartment_name: str

    class Config:
        orm_mode = True

    def __init__(
        self,
        type_guid: str = "",  # string
        quantity: int = 0,  # int32
        compartment_name: str = "",  # string
        **kwargs,
    ):
        super().__init__(
            type_guid=type_guid,
            quantity=quantity,
            compartment_name=compartment_name,
            **kwargs,
        )


# string type_guid
# int32 quantity
# string compartment_name
