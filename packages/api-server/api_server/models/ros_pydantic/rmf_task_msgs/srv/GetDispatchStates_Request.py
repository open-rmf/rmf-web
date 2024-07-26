# This is a generated file, do not edit

from typing import Annotated

import pydantic


class GetDispatchStates_Request(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    task_ids: list[str]
