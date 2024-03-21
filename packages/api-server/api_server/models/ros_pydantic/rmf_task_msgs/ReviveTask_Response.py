# This is a generated file, do not edit

from typing import Annotated

import pydantic


class ReviveTask_Response(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    success: bool  # bool


#
#
# # Confirmation that this service call is processed
# bool success
#
