# This is a generated file, do not edit

from typing import Annotated

import pydantic


class IngestorRequestItem(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    type_guid: str
    quantity: Annotated[int, pydantic.Field(ge=-2147483648, le=2147483647)]
    compartment_name: str
