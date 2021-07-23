# This is a generated file, do not edit

from typing import List

import pydantic


class DispenserRequestItem(pydantic.BaseModel):
    type_guid: str = ""  # string
    quantity: pydantic.conint(ge=-2147483648, le=2147483647) = 0  # int32
    compartment_name: str = ""  # string

    class Config:
        orm_mode = True


# string type_guid
# int32 quantity
# string compartment_name
