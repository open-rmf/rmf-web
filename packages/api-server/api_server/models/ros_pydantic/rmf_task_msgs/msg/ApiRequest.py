# This is a generated file, do not edit

from typing import Annotated

import pydantic


class ApiRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    json_msg: str
    request_id: str
