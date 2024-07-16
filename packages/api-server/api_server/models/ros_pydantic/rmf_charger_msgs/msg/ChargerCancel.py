# This is a generated file, do not edit

from typing import Annotated

import pydantic


class ChargerCancel(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    charger_name: str
    request_id: str
